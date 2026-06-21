# Teleop tuning knobs — `ur10e_proximity_teleop`

Live-tunable parameters for the UR10e teleop node. All take effect **immediately**
via `ros2 param set` (no rebuild). The **orientation** maps apply on your next
**A**-press re-anchor; the feel knobs apply on the next tick.

> ⚠️ Tune on the real robot with the e-stop in hand and the pendant speed slider low.

## The three feel knobs

```bash
# REACH — how far the robot moves per unit of hand motion (0.5–2.0)
ros2 param set /ur10e_proximity_teleop scale_factor 0.5

# SPEED CAP — max joint step per tick, ×30 ≈ rad/s. Lower = slower/safer (≤ 0.07 safe)
ros2 param set /ur10e_proximity_teleop max_joint_delta 0.05

# JITTER FILTER — EMA smoothing. 1.0 = off, lower = smoother but laggier (0.25–1.0)
ros2 param set /ur10e_proximity_teleop smoothing_alpha 0.5
```

| Knob | What it does | Safe range | Default |
|---|---|---|---|
| `scale_factor` | reach — robot motion per unit of hand motion | 0.5–2.0 | 1.0 |
| `max_joint_delta` | per-tick joint speed cap (×30 ≈ rad/s); clamps fast moves so the robot trails & catches up instead of protective-stopping | ≤ 0.07 | 0.05 |
| `smoothing_alpha` | EMA low-pass on the command — kills jitter; `1.0` = off, lower = smoother but more lag | 0.25–1.0 | 0.5 |

## "Very safe" tested setup

```bash
ros2 param set /ur10e_proximity_teleop max_joint_delta 0.008
ros2 param set /ur10e_proximity_teleop scale_factor 1.0
ros2 param set /ur10e_proximity_teleop smoothing_alpha 0.1
```

## Singularity guard (rarely touched)

```bash
# Holds the arm if IK asks for a jump bigger than this (rad) — anti flip-out near singularities
ros2 param set /ur10e_proximity_teleop max_ik_jump 1.0
```

## Orientation maps (apply on next A-press)

```bash
ros2 param set /ur10e_proximity_teleop ori_axis_map "x,z,y"
ros2 param set /ur10e_proximity_teleop world_frame_ori true
# leading-minus values need the -- guard so they're not parsed as a flag:
ros2 param set /ur10e_proximity_teleop pos_axis_map -- "-x,y,z"
```

`world_frame_ori true` was the key fix that made the TCP mirror how your hand
rotates in the room; with it, simple axis swaps in `ori_axis_map` behave intuitively.

## Confirm a value is actually live

```bash
ros2 param get /ur10e_proximity_teleop smoothing_alpha
```

If a `set` succeeds but "does nothing," check `ros2 param get` first. A missing or
stale value means **Terminal 1 is running an old build** — restart it, don't keep
tuning.
