#!/usr/bin/env python3
"""
policy_ping.py — "is the model alive?" probe for the openpi policy server.

Zero-risk: NO robot, NO cameras, NO ROS. It just connects to the openpi
WebsocketPolicyServer, sends a few DUMMY observations (correct keys + shapes), and
prints what comes back — so you can validate the whole inference path the moment the
server loads a checkpoint (even a half-trained 20k one), before going anywhere near
the robot.

It confirms:
  • the server is up and loaded the checkpoint
  • the observation KEY NAMES match (else the server returns an error string we print)
  • the action shape/values look sane (e.g. (horizon, 6))
  • round-trip latency over the LAN

Run in ~/eval_venv (has openpi-client):
  ~/eval_venv/bin/python scripts/policy_ping.py --policy-host 192.168.6.1 --policy-port 8000
If the keys are wrong, pass the right ones (ask the coworker / read the printed
server metadata):
  ... --openpi-wrist-key observation/wrist_image --openpi-scene-key observation/image \
      --openpi-state-key observation/state --openpi-prompt-key prompt --openpi-action-key actions
"""
import argparse, time
import numpy as np

# match the recorder/eval: wrist 640x480, scene 640x483, state = 6 joints + EE(7)
UR_HOME = [1.5708, -1.5708, 2.5307, -0.9599, 1.5708, 0.0]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--policy-host', default='192.168.6.1')
    p.add_argument('--policy-port', type=int, default=8000)
    p.add_argument('--task', default='open the fuel door')
    p.add_argument('--queries', type=int, default=5)
    p.add_argument('--openpi-wrist-key',  default='observation/wrist_image')
    p.add_argument('--openpi-scene-key',  default='observation/image')
    p.add_argument('--openpi-state-key',  default='observation/state')
    p.add_argument('--openpi-prompt-key', default='prompt')
    p.add_argument('--openpi-action-key', default='actions')
    p.add_argument('--wrist-hw', default='480x640', help='dummy wrist image HxW')
    p.add_argument('--scene-hw', default='483x640', help='dummy scene image HxW')
    args = p.parse_args()

    from openpi_client import websocket_client_policy

    print(f'[ping] connecting to ws://{args.policy_host}:{args.policy_port} …')
    client = websocket_client_policy.WebsocketClientPolicy(host=args.policy_host, port=args.policy_port)
    try:
        print('[ping] server metadata:', client.get_server_metadata())
    except Exception as e:
        print('[ping] (no server metadata):', e)

    def hw(s): h, w = s.split('x'); return int(h), int(w)
    wh, ww = hw(args.wrist_hw); sh, sw = hw(args.scene_hw)
    rng = np.random.default_rng(0)
    state = np.asarray(UR_HOME + [0.4, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)  # 13

    def make_obs():
        return {
            args.openpi_wrist_key: rng.integers(0, 256, (wh, ww, 3), dtype=np.uint8),
            args.openpi_scene_key: rng.integers(0, 256, (sh, sw, 3), dtype=np.uint8),
            args.openpi_state_key: state,
            args.openpi_prompt_key: args.task,
        }

    print(f'[ping] sending {args.queries} dummy observations '
          f'(keys: {args.openpi_wrist_key}, {args.openpi_scene_key}, {args.openpi_state_key}, '
          f'{args.openpi_prompt_key}) …')
    lats = []
    for i in range(args.queries):
        t0 = time.perf_counter()
        try:
            res = client.infer(make_obs())
        except Exception as e:
            print(f'\n[ping] ❌ inference FAILED: {e}')
            print('[ping] → likely an observation-key mismatch. Check the keys above '
                  'against the server metadata / ask the coworker, and re-run with the '
                  'right --openpi-*-key values.')
            return
        dt = (time.perf_counter() - t0) * 1000
        lats.append(dt)
        if args.openpi_action_key not in res:
            print(f'[ping] ⚠️ response has keys {list(res.keys())} but no '
                  f'"{args.openpi_action_key}" — set --openpi-action-key to the right one.')
            return
        act = np.asarray(res[args.openpi_action_key])
        if i == 0:
            print(f'[ping] ✅ got a response. action key "{args.openpi_action_key}" '
                  f'shape={act.shape} dtype={act.dtype}')
            print(f'[ping]    first action row: {np.atleast_2d(act)[0]}')
            print(f'[ping]    response keys: {list(res.keys())}')
        print(f'[ping]   query {i+1}/{args.queries}: {dt:.0f} ms')

    lats = np.array(lats)
    print(f'\n[ping] ✅ ALIVE. latency mean {lats.mean():.0f} ms, '
          f'max {lats.max():.0f} ms over {len(lats)} queries.')
    print('[ping] The inference path works. (A 20k-step checkpoint will still behave '
          'poorly — this only proves the pipeline is alive, not that the policy is good.)')


if __name__ == '__main__':
    main()
