#!/usr/bin/env python3
"""
episode_recorder.py — record teleop episodes into a LeRobot dataset.

This node is a pure FOLLOWER of the teleop node, which owns all the buttons:
  right A  : clutch (during an episode = pause/resume)
  right B  : start / stop episode   -> teleop emits /episode_event 'start' | 'save'
  left  X  : discard                -> teleop emits /episode_event 'discard'
  left  Y  : e-stop (discards)       -> teleop emits /episode_event 'discard'
The teleop publishes /record_active (Bool): True only while ENGAGED and an episode
is open. This node captures a frame ONLY while /record_active is True, so a clutch-
out pause adds no frames and the saved episode stays continuous (no gap, no jump).

Per frame (sampled at --fps):
  observation.images.wrist : RGB video    (D435i color)
  observation.images.scene : RGB video    (LUCID Triton, only with --lucid-topic)
  observation.state        : float32[13]  = 6 joint pos + EE pose (x,y,z,qx,qy,qz,qw)
  action                   : float32[6]   = commanded joint pos (teleop output)
Depth rides ALONGSIDE as lossless 16-bit PNG:
  <root>/depth/episode_<NNNNNN>/frame_<NNNNNN>.png  (index-aligned)

Voice cues via `spd-say`.

Run (in the lerobot venv, ROS env on PYTHONPATH):
  source /opt/ros/humble/setup.bash
  source ~/projects/LearnROS2/ros2_ws/install/setup.bash
  export ROS_DOMAIN_ID=69
  ~/lerobot_venv/bin/python src/Quest2ROS2/scripts/episode_recorder.py --task "open the box"
"""
import argparse
import os
import pathlib
import shutil
import subprocess

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Bool, String
from geometry_msgs.msg import PoseStamped

from PIL import Image as PILImage
from lerobot.datasets.lerobot_dataset import LeRobotDataset

UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
STATE_NAMES = [f'joint_{i}' for i in range(6)] + \
              ['ee_x', 'ee_y', 'ee_z', 'ee_qx', 'ee_qy', 'ee_qz', 'ee_qw']
ACTION_NAMES = [f'joint_{i}' for i in range(6)]


def image_to_rgb(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    enc = msg.encoding.lower()
    if enc == 'bgr8':
        return np.ascontiguousarray(buf[:, :, ::-1])
    if enc == 'rgb8':
        return np.ascontiguousarray(buf[:, :, :3])
    if enc in ('rgba8', 'bgra8'):
        rgb = buf[:, :, :3]
        return np.ascontiguousarray(rgb[:, :, ::-1] if enc == 'bgra8' else rgb)
    raise ValueError(f'unsupported color encoding: {msg.encoding}')


def depth_to_u16(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).copy()


class EpisodeRecorder(Node):
    def __init__(self, args):
        super().__init__('episode_recorder')
        self.args = args
        self.color = self.depth = self.joints = self.action = self.ee = None
        self.lucid = None
        self.use_lucid = bool(args.lucid_topic)
        self.record_active = False
        self.ep_open = False
        self.ep_idx = 0
        self.buffer = []                        # in-memory frames for the open episode
        self.dataset = None
        self.root = pathlib.Path(os.path.expanduser(args.root)).resolve()

        # Two callback groups under a MultiThreadedExecutor:
        #  - sensor subs run in PARALLEL (default group) so the 500 Hz /joint_states
        #    flood can't starve the capture timer.
        #  - the capture timer + episode-event handler share a MUTUALLY-EXCLUSIVE
        #    group so they never race on self.buffer / ep_open.
        cap = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, args.color_topic, lambda m: setattr(self, 'color', m), qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, lambda m: setattr(self, 'depth', m), qos_profile_sensor_data)
        self.create_subscription(JointState, args.joint_states_topic, self._js_cb, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, args.action_topic, lambda m: setattr(self, 'action', list(m.data)), 10)
        self.create_subscription(PoseStamped, args.ee_topic, self._ee_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, '/record_active', lambda m: setattr(self, 'record_active', m.data), 10)
        self.create_subscription(String, '/episode_event', self._event_cb, 10, callback_group=cap)
        if self.use_lucid:
            self.create_subscription(Image, args.lucid_topic, lambda m: setattr(self, 'lucid', m), qos_profile_sensor_data)

        self.create_timer(1.0 / args.fps, self._tick, callback_group=cap)
        self.get_logger().info(
            f'[recorder] ready @ {args.fps} Hz. Follows teleop: B=start/stop, X=discard, '
            f'Y=e-stop. task="{args.task}"  dataset={self.root}')
        self.speak('Recording session ready. Press B to start recording.')

    # ── voice ────────────────────────────────────────────────────────
    def speak(self, text):
        self.get_logger().info(f'[voice] {text}')
        if self.args.voice_type == 'off':
            return
        # spd-say: -l language, -t voice type (male1/2/3, female1..), -r rate (-100..100)
        cmd = ['spd-say', '-l', self.args.voice_lang, '-t', self.args.voice_type,
               '-r', str(self.args.voice_rate), text]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass

    # ── subscriptions ────────────────────────────────────────────────
    def _js_cb(self, m): self.joints = dict(zip(m.name, m.position))

    def _ee_cb(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.ee = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

    def _event_cb(self, m):
        {'start': self._begin, 'save': self._save, 'discard': self._discard}.get(
            m.data, lambda: self.get_logger().warn(f'[recorder] unknown event: {m.data}'))()

    # ── dataset ──────────────────────────────────────────────────────
    def _ensure_dataset(self):
        if self.dataset is not None:
            return True
        if self.color is None:
            self.get_logger().warn('[recorder] no color frame yet — cannot create dataset')
            return False
        if self.use_lucid and self.lucid is None:
            self.get_logger().warn('[recorder] no lucid frame yet — cannot create dataset '
                                   '(is lucid_camera_node running?)')
            return False
        h, w = self.color.height, self.color.width
        features = {
            'observation.images.wrist': {'dtype': 'video', 'shape': (h, w, 3),
                                         'names': ['height', 'width', 'channels']},
            'observation.state': {'dtype': 'float32', 'shape': (13,), 'names': STATE_NAMES},
            'action': {'dtype': 'float32', 'shape': (6,), 'names': ACTION_NAMES},
        }
        if self.use_lucid:
            lh, lw = self.lucid.height, self.lucid.width
            features['observation.images.scene'] = {'dtype': 'video', 'shape': (lh, lw, 3),
                                                    'names': ['height', 'width', 'channels']}
        # `meta/tasks.parquet` is written only after the FIRST save_episode(), so
        # it's the real marker of a resumable dataset. A dir with meta/ but no
        # tasks.parquet is a half-created leftover (create() ran, but every episode
        # was empty/discarded) — it can't be resumed (load fails -> falls back to
        # the HF hub -> 401), so wipe it and start clean.
        resumable = (self.root / 'meta' / 'tasks.parquet').exists()
        if resumable:
            self.dataset = LeRobotDataset(self.args.repo_id, root=str(self.root))
            # reload defaults to buffering 10 episodes of metadata and the teardown
            # flush is broken → resumed episodes wouldn't persist. Flush every episode.
            self.dataset.meta.metadata_buffer_size = 1
            self.get_logger().info(f'[recorder] resumed ({self.dataset.meta.total_episodes} episodes)')
        else:
            if self.root.exists():
                self.get_logger().warn(
                    f'[recorder] incomplete/empty dataset at {self.root} '
                    '(no saved episodes) — recreating')
                shutil.rmtree(self.root)
            self.dataset = LeRobotDataset.create(
                repo_id=self.args.repo_id, fps=self.args.fps, features=features,
                root=str(self.root), robot_type='ur10e', use_videos=True,
                metadata_buffer_size=1)
            self.get_logger().info('[recorder] created new dataset')
        return True

    def _begin(self):
        if self.ep_open:
            return
        if not self._ensure_dataset():
            self.speak('Cannot start, no camera yet.')
            return
        self.ep_idx = self.dataset.meta.total_episodes
        self.buffer = []                        # frames captured this episode; encoded at _save
        self.ep_open = True
        self.speak(f'Recording episode {self.ep_idx + 1}.')

    def _save(self):
        if not self.ep_open:
            return
        self.ep_open = False
        n = len(self.buffer)
        if n == 0:
            self.speak('Episode empty, nothing recorded.')
            self.get_logger().warn('[recorder] 0 frames — was the camera up and were you engaged?')
            return
        # Encode the whole episode here (NOT during capture) so the capture cadence
        # stayed steady. This blocks for a moment — that's fine, the episode is over.
        self.speak(f'Encoding episode {self.ep_idx + 1}, {n} frames. Please wait.')
        self.get_logger().info(f'[recorder] encoding {n} frames…')
        ddir = self.root / 'depth' / f'episode_{self.ep_idx:06d}'
        if ddir.exists():                       # clear stale depth from a prior discard
            shutil.rmtree(ddir)
        ddir.mkdir(parents=True, exist_ok=True)
        k = 0
        for (c, d, j, a, e, l) in self.buffer:
            try:                                # convert raw msgs -> arrays at encode time
                rgb = image_to_rgb(c)
                state = np.asarray([float(j[nm]) for nm in UR_JOINTS] + list(e), dtype=np.float32)
                action = np.asarray([float(x) for x in a[:6]], dtype=np.float32)
                depth = depth_to_u16(d)
                scene = image_to_rgb(l) if self.use_lucid else None
            except (KeyError, ValueError, IndexError) as ex:
                self.get_logger().warn(f'[recorder] frame skipped at encode: {ex}')
                continue
            frame = {'observation.images.wrist': rgb,
                     'observation.state': state,
                     'action': action,
                     'task': self.args.task}
            if scene is not None:
                frame['observation.images.scene'] = scene
            self.dataset.add_frame(frame)
            PILImage.fromarray(depth, mode='I;16').save(ddir / f'frame_{k:06d}.png')
            k += 1
        if k == 0:
            if getattr(self.dataset, 'episode_buffer', None) is not None:
                self.dataset.clear_episode_buffer()
            self.buffer = []
            self.speak('Episode empty, nothing recorded.')
            return
        self.dataset.save_episode()
        self.buffer = []
        self.speak(f'Episode {self.ep_idx + 1} saved.')
        self.get_logger().info(f'[recorder] ✓ total episodes: {self.dataset.meta.total_episodes}')

    def _discard(self):
        if not self.ep_open:
            return
        self.ep_open = False
        self.buffer = []                        # captured frames live only in memory until _save
        self.speak('Episode discarded.')

    # ── frame capture ────────────────────────────────────────────────
    def _tick(self):
        if not (self.ep_open and self.record_active):
            return
        # Snapshot the latest RAW messages (atomic reference reads — no conversion,
        # no disk). This keeps the callback ~free so the 10 Hz cadence is exact even
        # while /joint_states floods at 500 Hz on other executor threads. All
        # conversion + PNG/add_frame work is batched in _save().
        c, d, j, a, e, l = (self.color, self.depth, self.joints,
                            self.action, self.ee, self.lucid)
        streams = [('color', c), ('depth', d), ('joints', j), ('action', a), ('ee', e)]
        if self.use_lucid:
            streams.append(('lucid', l))
        missing = [n for n, v in streams if v is None]
        if missing:
            self.get_logger().warn(f'[recorder] recording but missing: {missing}',
                                   throttle_duration_sec=2.0)
            return
        self.buffer.append((c, d, j, a, e, l))
        if len(self.buffer) % (self.args.fps * 5) == 0:
            self.get_logger().info(f'[recorder]   …{len(self.buffer)} frames')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--repo-id', default='ur10e/teleop')
    p.add_argument('--root', default='~/lerobot_datasets/ur10e_teleop')
    p.add_argument('--task', default='teleop episode')
    p.add_argument('--fps', type=int, default=30)
    p.add_argument('--lucid-topic', default='',
                   help='secondary scene camera topic (e.g. /lucid/image_raw); '
                        'empty = single-camera. Adds observation.images.scene — '
                        'this CHANGES the schema, so use a fresh --root.')
    # voice cues (spd-say / espeak-ng). voice-type: male1/male2/male3/female1.. or 'off'
    p.add_argument('--voice-type', default='male2',
                   help="spd-say voice type: male1/male2/male3/female1.. , or 'off' to mute")
    p.add_argument('--voice-lang', default='en-US', help='spd-say language (e.g. en-US)')
    p.add_argument('--voice-rate', type=int, default=-10, help='spd-say rate -100..100 (slower<0)')
    p.add_argument('--color-topic', default='/camera/camera/color/image_raw')
    p.add_argument('--depth-topic', default='/camera/camera/aligned_depth_to_color/image_raw')
    p.add_argument('--joint-states-topic', dest='joint_states_topic', default='/joint_states')
    p.add_argument('--action-topic', default='/forward_position_controller/commands')
    p.add_argument('--ee-topic', default='/tcp_pose_broadcaster/pose')
    args = p.parse_args()

    rclpy.init()
    node = EpisodeRecorder(args)
    # MultiThreadedExecutor: sensor callbacks run in parallel so the 500 Hz
    # /joint_states stream can't starve the capture timer (which lives in its own
    # mutually-exclusive group with the episode-event handler).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        if node.ep_open:
            node._discard()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
