"""Standalone check: build the ikpy chain for HC10DTP exactly like
hc10dtp_teleop_controller.py does, then round-trip FK -> IK."""
import os
import tempfile
import numpy as np
import xacro
import ikpy.chain
from ament_index_python.packages import get_package_share_directory

support_dir = get_package_share_directory('motoman_hc10_support')
xacro_file = os.path.join(support_dir, 'urdf', 'hc10dtp_b00.xacro')
print(f"Xacro: {xacro_file}")

doc = xacro.process_file(xacro_file)
urdf_xml = doc.toxml()

with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
    f.write(urdf_xml)
    temp_urdf = f.name

elements = [
    "base_link",
    "joint_1", "link_1",
    "joint_2", "link_2",
    "joint_3", "link_3",
    "joint_4", "link_4",
    "joint_5", "link_5",
    "joint_6", "link_6",
    "joint_6-flange", "flange",
    "flange-tool0", "tool0",
]
mask = [False, True, True, True, True, True, True, False, False]

chain = ikpy.chain.Chain.from_urdf_file(
    temp_urdf, base_elements=elements, active_links_mask=mask)

print(f"\nChain has {len(chain.links)} links:")
for i, link in enumerate(chain.links):
    print(f"  [{i}] {link.name}  active={chain.active_links_mask[i]}")

# FK at a non-trivial pose
q_true = np.zeros(len(chain.links))
q_true[1:7] = [0.3, -0.4, 0.5, 0.2, -0.6, 0.1]
T = chain.forward_kinematics(q_true)
pos = T[:3, 3]
rot = T[:3, :3]
print(f"\nFK tool0 position at test config: {pos}")

# IK seeded near a slightly different config
seed = np.zeros(len(chain.links))
seed[1:7] = [0.25, -0.35, 0.45, 0.15, -0.55, 0.05]
import time
t0 = time.time()
q_ik = chain.inverse_kinematics(
    target_position=pos, target_orientation=rot,
    orientation_mode="all", initial_position=seed)
dt = (time.time() - t0) * 1000

T2 = chain.forward_kinematics(q_ik)
pos_err = np.linalg.norm(T2[:3, 3] - pos)
rot_err = np.linalg.norm(T2[:3, :3] - rot)
print(f"\nIK solve time: {dt:.1f} ms")
print(f"IK joints: {np.round(q_ik[1:7], 4)}")
print(f"True     : {np.round(q_true[1:7], 4)}")
print(f"Position error: {pos_err*1000:.3f} mm, rotation error (fro): {rot_err:.5f}")

os.remove(temp_urdf)
ok = pos_err < 0.002 and rot_err < 0.01
print("\nRESULT:", "PASS" if ok else "FAIL")
