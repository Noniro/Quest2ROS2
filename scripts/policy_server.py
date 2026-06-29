#!/usr/bin/env python3
"""
policy_server.py — REFERENCE inference server for evaluating a trained π0 (or ACT/
diffusion) policy on the UR10e over the LAN.

RUNS ON THE GPU MACHINE (NOT the robot laptop — that laptop has no GPU). The robot
laptop runs scripts/policy_eval.py, which POSTs observations here and gets actions
back. Both must be on the same network.

  robot laptop (policy_eval.py)  ──HTTP/LAN──>  this server (GPU)  ──>  π0 checkpoint
        cameras + joints  ─────────────────────────────────────────>  action chunk
                          <─────────────────────────────────────────

────────────────────────────────────────────────────────────────────────────────
WIRE CONTRACT  (must match scripts/policy_eval.py :: RemotePolicyClient)
────────────────────────────────────────────────────────────────────────────────
POST /act   request JSON:
    {
      "images": { "wrist": "<base64 JPEG>",   # D435i color, ~640x480
                  "scene": "<base64 JPEG>" },  # LUCID,       ~640x483
      "state":  [13 floats],                   # 6 joint pos (UR order) + EE xyz+quat
      "task":   "open the fuel door"           # language instruction
    }
            response JSON:
    { "action": [[6 floats], [6 floats], ...] }  # an ACTION CHUNK, T x 6
            where each row = ABSOLUTE joint positions (UR order), to be executed at
            10 Hz (the dataset fps). Returning the whole chunk (not 1 action) is what
            hides network/inference latency — please do this.

POST /reset  (optional) — clear any per-episode policy state. Called once per trial.

────────────────────────────────────────────────────────────────────────────────
MUST MATCH TRAINING (these are the usual failure points):
  • observation keys:  observation.images.wrist, observation.images.scene,
                       observation.state   (exactly these names)
  • state layout (13): [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2,
                        wrist_3]  +  [ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw]
  • action (6):        ABSOLUTE joint targets in the same UR joint order (NOT deltas)
  • image preprocessing & normalization: use the policy's OWN input transforms /
    the dataset normalization stats from training — do not hand-roll different
    resize/normalize, or the policy will silently underperform.
────────────────────────────────────────────────────────────────────────────────

Run:
    pip install flask torch  lerobot   # on the GPU box, your training env
    python policy_server.py --checkpoint /path/to/ckpt_folder --host 0.0.0.0 --port 9000
Then tell the robot operator the IP:port; they run:
    eval_session.sh "open the fuel door" fuel_door --policy-host http://<this-ip>:9000
"""
import argparse, base64, io

import numpy as np
from flask import Flask, request, jsonify

OBS_IMG_WRIST = "observation.images.wrist"
OBS_IMG_SCENE = "observation.images.scene"
OBS_STATE     = "observation.state"


def build_app(checkpoint: str, device: str = "cuda"):
    import torch
    import cv2

    # ── load the policy ───────────────────────────────────────────────────────
    # TODO(coworker): adjust import/loader to your LeRobot version & policy type.
    # For π0 in LeRobot 0.4.x this is typically:
    from lerobot.policies.pi0.modeling_pi0 import PI0Policy
    policy = PI0Policy.from_pretrained(checkpoint)
    policy.to(device).eval()

    app = Flask(__name__)

    def decode(b64):                       # base64 JPEG -> HWC uint8 RGB
        buf = np.frombuffer(base64.b64decode(b64), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def to_img_tensor(rgb):                # HWC uint8 -> 1,C,H,W float[0,1] on device
        t = torch.from_numpy(rgb).permute(2, 0, 1)[None].float() / 255.0
        return t.to(device)
        # NOTE: if your training resized images, the policy's normalize_inputs should
        # handle it; otherwise resize HERE to the training resolution before this.

    @app.post("/reset")
    def reset():
        if hasattr(policy, "reset"):
            policy.reset()
        return jsonify(ok=True)

    @app.post("/act")
    def act():
        data = request.get_json(force=True)
        batch = {
            OBS_IMG_WRIST: to_img_tensor(decode(data["images"]["wrist"])),
            OBS_IMG_SCENE: to_img_tensor(decode(data["images"]["scene"])),
            OBS_STATE: torch.tensor(data["state"], dtype=torch.float32)[None].to(device),
            "task": [data.get("task", "")],
        }
        with torch.no_grad():
            # PREFERRED: return the whole horizon so latency is hidden.
            # TODO(coworker): use whichever your version exposes:
            if hasattr(policy, "predict_action_chunk"):
                chunk = policy.predict_action_chunk(batch)      # (1, T, 6)
                actions = chunk.squeeze(0).cpu().numpy()
            else:
                # fallback: single action per call (robot will re-query every tick;
                # fine on wired, can stutter on WiFi)
                a = policy.select_action(batch)                 # (1, 6)
                actions = a.cpu().numpy()
        actions = np.atleast_2d(actions).astype(float)          # ensure (T, 6)
        return jsonify(action=actions.tolist())

    return app


def main():
    p = argparse.ArgumentParser(description="UR10e policy inference server (LAN).")
    p.add_argument("--checkpoint", required=True, help="path to the trained checkpoint folder")
    p.add_argument("--host", default="0.0.0.0", help="0.0.0.0 = reachable from the LAN")
    p.add_argument("--port", type=int, default=9000)
    p.add_argument("--device", default="cuda")
    args = p.parse_args()
    app = build_app(args.checkpoint, args.device)
    # threaded=False keeps GPU inference serialized (one robot client anyway)
    app.run(host=args.host, port=args.port, threaded=False)


if __name__ == "__main__":
    main()
