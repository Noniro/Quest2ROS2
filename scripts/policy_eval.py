#!/usr/bin/env python3
"""
policy_eval.py — autonomous evaluation harness for a trained policy on the UR10e.

This is the MIRROR of episode_recorder.py:
  recorder:  human (teleop)  -> robot -> SAVE observations
  eval:      trained policy  -> robot -> MEASURE success

It runs N trials in a loop. Each trial:
  1. HOMING     — robot auto-returns to UR_HOME (this node drives it; no teleop needed).
  2. WAIT_READY — you reset the scene (close the fuel door, vary the start), press READY.
  3. RUNNING    — the policy drives the robot: obs (wrist+scene+state) -> action ->
                  /forward_position_controller/commands, until a time limit or you press STOP.
  4. JUDGING    — you mark SUCCESS / FAIL (the policy does NOT self-judge in v1).
Then it logs the trial (result + a wrist-cam video) and loops. At the end it prints
the success rate and flags the failures so you know what to collect next.

Controls (Quest controllers, same headset as teleop; or --input keyboard for desk tests):
  RIGHT A (lower) = READY / go            RIGHT B (upper) = STOP this run early
  LEFT  X (lower) = mark SUCCESS          LEFT  Y (upper) = mark FAIL / abort
The PHYSICAL pendant e-stop is the real safety — keep a hand on it; the policy drives
the real arm autonomously. A per-tick joint-delta clamp limits how far each command can
jump (rejects wild policy outputs).

SERVING THE POLICY (the one thing to confirm with your coworker):
  --serve remote  (default) : POST observations to a policy server over the network
                              (this laptop has no GPU). See RemotePolicyClient — the wire
                              format is a PLACEHOLDER; match it to the actual server.
  --serve local             : load a LeRobot checkpoint and run inference here (needs a GPU).

Run (cameras must be up — use eval_session.sh which launches them first):
  source /opt/ros/humble/setup.bash
  source ~/projects/LearnROS2/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=69
  ~/lerobot_venv/bin/python src/Quest2ROS2/scripts/policy_eval.py \
      --serve remote --policy-host http://10.0.0.x:PORT --trials 20 --task "open the fuel door"
"""
import argparse, base64, csv, json, os, pathlib, subprocess, threading, time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

# Quest inputs are optional (only with --input quest)
try:
    from quest2ros.msg import OVR2ROSInputs
except Exception:
    OVR2ROSInputs = None

# ── must MATCH episode_recorder.py exactly (the model was trained on these) ──
UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
UR_HOME = {'shoulder_pan_joint': 1.5708, 'shoulder_lift_joint': -1.5708,
           'elbow_joint': 2.5307, 'wrist_1_joint': -0.9599,
           'wrist_2_joint': 1.5708, 'wrist_3_joint': 0.0}


def image_to_rgb(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    enc = msg.encoding.lower()
    if enc == 'bgr8':   return np.ascontiguousarray(buf[:, :, ::-1])
    if enc == 'rgb8':   return np.ascontiguousarray(buf[:, :, :3])
    if enc in ('rgba8', 'bgra8'):
        rgb = buf[:, :, :3]
        return np.ascontiguousarray(rgb[:, :, ::-1] if enc == 'bgra8' else rgb)
    raise ValueError(f'unsupported color encoding: {msg.encoding}')


# ─────────────────────────── policy clients ────────────────────────────────
class PolicyClient:
    """An action source. select_action() may return a single action (6,) or a
    chunk (T, 6) — the node plays a chunk out one step per tick, then re-queries."""
    def reset(self): pass
    def select_action(self, obs: dict) -> np.ndarray: raise NotImplementedError


class RemotePolicyClient(PolicyClient):
    """Sends observations to a policy server over the network and gets actions back.

    ⚠️ PLACEHOLDER WIRE FORMAT — confirm/adjust to your coworker's server. Right now it:
      POST {host}/reset                         (optional, on each trial)
      POST {host}/act  with JSON:
          {"images": {"wrist": <b64 jpeg>, "scene": <b64 jpeg>},
           "state":  [13 floats],
           "task":   "<language instruction>"}
      expects JSON back: {"action": [6 floats]}   OR  {"action": [[6],[6],...]}  (a chunk)
    If the server speaks LeRobot's own async-inference / websocket protocol instead,
    swap this method's body — everything else in the harness stays the same.
    """
    def __init__(self, host, task, timeout=5.0):
        import requests  # local import so 'local' serving needn't have it
        self._requests = requests
        self.host = host.rstrip('/')
        self.task = task
        self.timeout = timeout

    def _jpeg_b64(self, rgb):
        import cv2
        ok, buf = cv2.imencode('.jpg', rgb[:, :, ::-1])  # cv2 wants BGR
        if not ok: raise RuntimeError('jpeg encode failed')
        return base64.b64encode(buf.tobytes()).decode('ascii')

    def reset(self):
        try:
            self._requests.post(f'{self.host}/reset', timeout=self.timeout)
        except Exception:
            pass  # server may not implement /reset

    def select_action(self, obs):
        payload = {
            'images': {'wrist': self._jpeg_b64(obs['images']['wrist']),
                       'scene': self._jpeg_b64(obs['images']['scene'])},
            'state': [float(x) for x in obs['state']],
            'task': self.task,
        }
        r = self._requests.post(f'{self.host}/act', json=payload, timeout=self.timeout)
        r.raise_for_status()
        return np.asarray(r.json()['action'], dtype=np.float32)


class LocalPolicyClient(PolicyClient):
    """Loads a LeRobot checkpoint and runs inference in-process. Needs torch + a GPU
    (this laptop has neither, so this is a stub for when run on a stronger machine)."""
    def __init__(self, ckpt_path, task, device='cuda'):
        import torch
        from lerobot.policies.factory import make_policy  # path may differ per lerobot ver
        self.torch = torch
        self.device = device
        self.task = task
        # TODO: load exactly as your training config did; this is the common shape.
        self.policy = make_policy(ckpt_path)  # adjust to the real loader/signature
        self.policy.to(device).eval()

    def reset(self):
        if hasattr(self.policy, 'reset'): self.policy.reset()

    def select_action(self, obs):
        torch = self.torch
        def img(x):  # HWC uint8 -> 1,C,H,W float[0,1]
            t = torch.from_numpy(x).permute(2, 0, 1)[None].float() / 255.0
            return t.to(self.device)
        batch = {
            'observation.images.wrist': img(obs['images']['wrist']),
            'observation.images.scene': img(obs['images']['scene']),
            'observation.state': torch.tensor(obs['state'], dtype=torch.float32)[None].to(self.device),
            'task': [self.task],
        }
        with torch.no_grad():
            act = self.policy.select_action(batch)
        return act.squeeze(0).cpu().numpy().astype(np.float32)


class OpenPiClient(PolicyClient):
    """Talks to an openpi WebsocketPolicyServer (Physical Intelligence's π0 serving).

    Run in ~/eval_venv (openpi-client pins numpy<2, so it canNOT live in lerobot_venv).
    openpi serializes raw numpy arrays over the wire (msgpack) — so we send the RAW RGB
    images + state, NO jpeg encoding. client.infer(obs) -> {"actions": (horizon, 6), ...}.

    ⚠️ The observation KEY NAMES are defined by the coworker's openpi data/repack config,
    NOT by us. Defaults below are common openpi names — CONFIRM the exact keys (and image
    size / state layout / action key) with whoever trained & serves the model. The server
    also announces metadata on connect, which we log to help you discover the keys.
    """
    def __init__(self, host, port, task, keys):
        from openpi_client import websocket_client_policy
        self.client = websocket_client_policy.WebsocketClientPolicy(host=host, port=port)
        self.task = task
        self.k = keys  # dict: wrist, scene, state, prompt, action
        try:
            print('[openpi] server metadata:', self.client.get_server_metadata())
        except Exception:
            pass

    def reset(self):
        try: self.client.reset()
        except Exception: pass

    def select_action(self, obs):
        payload = {
            self.k['wrist']: obs['images']['wrist'],          # raw HWC uint8 RGB
            self.k['scene']: obs['images']['scene'],
            self.k['state']: np.asarray(obs['state'], dtype=np.float32),
            self.k['prompt']: self.task,
        }
        res = self.client.infer(payload)
        return np.asarray(res[self.k['action']], dtype=np.float32)  # (horizon, 6) or (6,)


# ─────────────────────────── the eval node ─────────────────────────────────
HOMING, WAIT_READY, RUNNING, JUDGING, DONE = 'HOMING', 'WAIT_READY', 'RUNNING', 'JUDGING', 'DONE'


class PolicyEval(Node):
    def __init__(self, args, policy):
        super().__init__('policy_eval')
        self.args = args
        self.policy = policy
        self.color = self.lucid = self.joints = self.ee = None
        self.rate = float(args.rate)
        self.home_step = np.radians(args.home_speed_deg_s) / self.rate

        # control-button edge flags
        self.f = {'ready': False, 'stop': False, 'success': False, 'fail': False}

        cap = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, args.color_topic, lambda m: setattr(self, 'color', m), qos_profile_sensor_data)
        self.create_subscription(Image, args.lucid_topic, lambda m: setattr(self, 'lucid', m), qos_profile_sensor_data)
        self.create_subscription(JointState, args.joint_states_topic, self._js_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, args.ee_topic, self._ee_cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Float64MultiArray, args.action_topic, 10)

        if args.input == 'quest':
            if OVR2ROSInputs is None:
                raise RuntimeError('quest2ros msgs not found — source the workspace, or use --input keyboard')
            self._r = {'lower': False, 'upper': False}
            self._l = {'lower': False, 'upper': False}
            self.create_subscription(OVR2ROSInputs, '/q2r_right_hand_inputs', self._right_cb, 10)
            self.create_subscription(OVR2ROSInputs, '/q2r_left_hand_inputs', self._left_cb, 10)
        else:
            threading.Thread(target=self._keyboard_loop, daemon=True).start()

        # results
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.outdir = pathlib.Path(os.path.expanduser(args.out)).resolve() / f'{args.dataset}_{stamp}'
        self.outdir.mkdir(parents=True, exist_ok=True)
        self.results = []
        self.trial = 0
        self.state = HOMING
        self.last_cmd = None
        self.video_frames = []
        self.run_start = 0.0
        self._action_queue = []
        self._q_latencies = []                       # per-query round-trip ms (incl. network)
        self._tick_budget_ms = 1000.0 / self.rate

        self.create_timer(1.0 / self.rate, self._tick, callback_group=cap)
        self.get_logger().info(f'[eval] {args.trials} trials @ {self.rate} Hz, serve={args.serve}, '
                               f'input={args.input}. Results -> {self.outdir}')
        self.speak(f'Evaluation ready. {args.trials} trials. Homing.')

    # ── io ──
    def _js_cb(self, m): self.joints = dict(zip(m.name, m.position))
    def _ee_cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.ee = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    def _right_cb(self, m):
        if m.button_lower and not self._r['lower']: self.f['ready'] = True
        if m.button_upper and not self._r['upper']: self.f['stop'] = True
        self._r['lower'], self._r['upper'] = bool(m.button_lower), bool(m.button_upper)
    def _left_cb(self, m):
        if m.button_lower and not self._l['lower']: self.f['success'] = True
        if m.button_upper and not self._l['upper']: self.f['fail'] = True
        self._l['lower'], self._l['upper'] = bool(m.button_lower), bool(m.button_upper)

    def _keyboard_loop(self):
        hint = "[keys] r=ready/go  s=stop  t=success  f=fail  (letter + Enter) > "
        while True:
            try: c = input(hint).strip().lower()
            except (EOFError, OSError): return
            self.f['ready']  |= c == 'r'
            self.f['stop']   |= c == 's'
            self.f['success']|= c == 't'
            self.f['fail']   |= c == 'f'

    def _take(self, key):
        v = self.f[key]; self.f[key] = False; return v

    def speak(self, text):
        self.get_logger().info(f'[eval] {text}')
        if self.args.voice_type == 'off': return
        try:
            subprocess.Popen(['spd-say', '-l', self.args.voice_lang, '-t', self.args.voice_type,
                              '-r', str(self.args.voice_rate), text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass

    # ── safety: clip each commanded joint to within home_step*K / max_joint_step of last cmd ──
    def _publish_clamped(self, target6):
        cur = np.array([self.joints[n] for n in UR_JOINTS]) if self.joints else target6
        base = self.last_cmd if self.last_cmd is not None else cur
        step = self.args.max_joint_step
        cmd = np.clip(np.asarray(target6, float), base - step, base + step)
        self.last_cmd = cmd
        self.cmd_pub.publish(Float64MultiArray(data=[float(x) for x in cmd]))

    def _obs(self):
        return {'images': {'wrist': image_to_rgb(self.color), 'scene': image_to_rgb(self.lucid)},
                'state': np.asarray([self.joints[n] for n in UR_JOINTS] + self.ee, dtype=np.float32)}

    def _sensors_ready(self):
        return None not in (self.color, self.lucid, self.joints, self.ee)

    # ── main loop ──
    def _tick(self):
        if self.state == DONE:
            return
        if not self._sensors_ready():
            return  # wait for cameras + joints + EE

        if self.state == HOMING:
            self._do_homing()
        elif self.state == WAIT_READY:
            if self._take('ready'):
                self.trial += 1
                self.policy.reset()
                self.last_cmd = np.array([self.joints[n] for n in UR_JOINTS])
                self.video_frames = []
                self._action_queue = []
                self.run_start = time.time()
                self.state = RUNNING
                self.speak(f'Trial {self.trial} running.')
        elif self.state == RUNNING:
            self._do_running()
        elif self.state == JUDGING:
            self._do_judging()

    def _do_homing(self):
        cur = np.array([self.joints[n] for n in UR_JOINTS])
        goal = np.array([UR_HOME[n] for n in UR_JOINTS])
        diff = goal - cur
        if np.max(np.abs(diff)) < 0.01:
            self.last_cmd = goal
            self.state = WAIT_READY
            self.speak(f'Homed. Reset the scene and press r for trial {self.trial + 1}.')
            return
        step = np.clip(diff, -self.home_step, self.home_step)
        self._publish_clamped(cur + step)  # gentle; clamp is a no-op here

    def _do_running(self):
        # buffer wrist frame for the trial video
        try: self.video_frames.append(image_to_rgb(self.color).copy())
        except Exception: pass

        # action chunking: re-query only when the queue is empty
        if not len(self._action_queue):
            try:
                t0 = time.perf_counter()
                act = self.policy.select_action(self._obs())
                dt_ms = (time.perf_counter() - t0) * 1000.0
            except Exception as e:
                self.get_logger().error(f'[eval] policy query failed: {e}')
                self.speak('Policy error. Stopping run.')
                self._end_run(); return
            act = np.atleast_2d(act)            # (6,)->(1,6); (T,6) stays
            self._q_latencies.append(dt_ms)
            # A query stalls the loop until it returns. If the chunk it returns
            # covers more time than the query took, latency is fully hidden; warn
            # only when a single-step query can't keep up with the control rate.
            if len(act) == 1 and dt_ms > self._tick_budget_ms:
                self.get_logger().warn(
                    f'[eval] query {dt_ms:.0f} ms > {self._tick_budget_ms:.0f} ms tick budget '
                    f'and server returned only 1 action — robot will stutter. Ask the server '
                    f'to return an action CHUNK to hide latency.')
            self._action_queue = [a for a in act]
        self._publish_clamped(self._action_queue.pop(0)[:6])

        if self._take('stop') or (time.time() - self.run_start) >= self.args.run_seconds:
            self._end_run()

    def _end_run(self):
        self.state = JUDGING
        self.speak('Run done. Press t for success or f for fail.')

    def _do_judging(self):
        res = None
        if self._take('success'): res = 'success'
        elif self._take('fail'):  res = 'fail'
        if res is None:
            return
        self._save_trial(res)
        if self.trial >= self.args.trials:
            self._finish()
        else:
            self.state = HOMING
            self.speak(f'Logged {res}. Homing for next trial.')

    # ── logging ──
    def _save_trial(self, result):
        vid = self.outdir / f'trial_{self.trial:03d}_{result}.mp4'
        self._write_video(vid, self.video_frames)
        dur = round(time.time() - self.run_start, 1)
        self.results.append({'trial': self.trial, 'result': result,
                             'seconds': dur, 'frames': len(self.video_frames),
                             'video': vid.name})
        with open(self.outdir / 'results.csv', 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=['trial', 'result', 'seconds', 'frames', 'video'])
            w.writeheader(); w.writerows(self.results)
        self.get_logger().info(f'[eval] trial {self.trial}: {result.upper()} ({dur}s) -> {vid.name}')

    def _write_video(self, path, frames):
        if not frames: return
        try:
            import cv2
            h, w = frames[0].shape[:2]
            vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), self.rate, (w, h))
            for fr in frames: vw.write(fr[:, :, ::-1])  # RGB->BGR
            vw.release()
        except Exception as e:
            self.get_logger().warn(f'[eval] could not write video: {e}')

    def _finish(self):
        self.state = DONE
        n = len(self.results)
        ok = sum(r['result'] == 'success' for r in self.results)
        fails = [r['trial'] for r in self.results if r['result'] == 'fail']
        self.get_logger().info('=' * 56)
        self.get_logger().info(f'[eval] SUCCESS RATE: {ok}/{n} = {100*ok/max(n,1):.0f}%')
        self.get_logger().info(f'[eval] failed trials: {fails or "none"}  (review their videos in {self.outdir})')
        if self._q_latencies:
            lat = np.array(self._q_latencies)
            self.get_logger().info(f'[eval] policy query latency: mean {lat.mean():.0f} ms, '
                                   f'p95 {np.percentile(lat,95):.0f} ms, max {lat.max():.0f} ms '
                                   f'(tick budget {self._tick_budget_ms:.0f} ms)')
        self.get_logger().info('=' * 56)
        self.speak(f'Evaluation complete. {ok} of {n} succeeded.')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--task', default='open the fuel door', help='language instruction given to the policy')
    p.add_argument('--dataset', default='fuel_door', help='label for the results folder')
    p.add_argument('--trials', type=int, default=20)
    p.add_argument('--run-seconds', type=float, default=20.0, help='max seconds per trial')
    p.add_argument('--rate', type=float, default=10.0, help='control Hz (match training fps)')
    # serving
    p.add_argument('--serve', choices=['openpi', 'remote', 'local'], default='openpi',
                   help='openpi=websocket server (default); remote=plain HTTP; local=in-process')
    p.add_argument('--policy-host', default='10.0.0.78', help='policy server host/IP')
    p.add_argument('--policy-port', type=int, default=8000, help='policy server port (openpi default 8000)')
    p.add_argument('--checkpoint', default='', help='local LeRobot checkpoint path (--serve local)')
    p.add_argument('--device', default='cuda')
    # openpi observation key names — CONFIRM with the coworker's openpi data config
    p.add_argument('--openpi-wrist-key',  default='observation/wrist_image')
    p.add_argument('--openpi-scene-key',  default='observation/image')
    p.add_argument('--openpi-state-key',  default='observation/state')
    p.add_argument('--openpi-prompt-key', default='prompt')
    p.add_argument('--openpi-action-key', default='actions')
    # safety
    p.add_argument('--max-joint-step', type=float, default=0.10, help='rad: max change per joint per tick')
    p.add_argument('--home-speed-deg-s', type=float, default=8.0)
    # io
    p.add_argument('--input', choices=['quest', 'keyboard'], default='keyboard')
    p.add_argument('--out', default='~/eval_runs')
    p.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    p.add_argument('--lucid-topic', default='/lucid/image_raw')
    p.add_argument('--joint-states-topic', dest='joint_states_topic', default='/joint_states')
    p.add_argument('--ee-topic', default='/tcp_pose_broadcaster/pose')
    p.add_argument('--action-topic', default='/forward_position_controller/commands')
    # voice
    p.add_argument('--voice-type', default='male2')
    p.add_argument('--voice-lang', default='en-US')
    p.add_argument('--voice-rate', type=int, default=-10)
    args = p.parse_args()

    if args.serve == 'openpi':
        keys = {'wrist': args.openpi_wrist_key, 'scene': args.openpi_scene_key,
                'state': args.openpi_state_key, 'prompt': args.openpi_prompt_key,
                'action': args.openpi_action_key}
        policy = OpenPiClient(args.policy_host, args.policy_port, args.task, keys)
    elif args.serve == 'remote':
        policy = RemotePolicyClient(args.policy_host, args.task)
    else:
        if not args.checkpoint:
            raise SystemExit('--serve local requires --checkpoint')
        policy = LocalPolicyClient(args.checkpoint, args.task, args.device)

    rclpy.init()
    node = PolicyEval(args, policy)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
