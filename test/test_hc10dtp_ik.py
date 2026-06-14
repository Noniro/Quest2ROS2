"""IK verification: build the ikpy chain exactly like hc10dtp_teleop_controller
does (URDF-derived chain spec) and round-trip FK -> IK.

Tests every HC10DTP support package found in the environment:
- hc10dtp_b00_support  (team package, joints joint_1_s..joint_6_t)
- motoman_hc10_support (upstream Yaskawa, joints joint_1..joint_6)
"""
import os
import sys
import tempfile
import time

import numpy as np
import xacro
import ikpy.chain
from ament_index_python.packages import get_package_share_directory

from q2r2_bringup.hc10dtp_teleop_controller import derive_chain_spec


def run_roundtrip(pkg):
    xacro_file = os.path.join(
        get_package_share_directory(pkg), 'urdf', 'hc10dtp_b00.xacro')
    print(f"\n=== {pkg} ===\nXacro: {xacro_file}")

    urdf_xml = xacro.process_file(xacro_file).toxml()
    elements, mask, joint_names = derive_chain_spec(urdf_xml)
    print(f"Derived joints: {joint_names}")
    print(f"Active mask   : {mask}")

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(urdf_xml)
        temp_urdf = f.name
    try:
        chain = ikpy.chain.Chain.from_urdf_file(
            temp_urdf, base_elements=elements, active_links_mask=mask)
    finally:
        os.remove(temp_urdf)

    active_idx = [i for i, a in enumerate(mask) if a]

    # FK at a non-trivial pose
    q_true = np.zeros(len(chain.links))
    test_vals = [0.3, -0.4, 0.5, 0.2, -0.6, 0.1]
    for idx, val in zip(active_idx, test_vals):
        q_true[idx] = val
    T = chain.forward_kinematics(q_true)
    pos, rot = T[:3, 3], T[:3, :3]
    print(f"FK tool0 position at test config: {np.round(pos, 4)}")

    # IK seeded near a slightly different config
    seed = np.zeros(len(chain.links))
    seed_vals = [0.25, -0.35, 0.45, 0.15, -0.55, 0.05]
    for idx, val in zip(active_idx, seed_vals):
        seed[idx] = val

    t0 = time.time()
    q_ik = chain.inverse_kinematics(
        target_position=pos, target_orientation=rot,
        orientation_mode="all", initial_position=seed)
    dt = (time.time() - t0) * 1000

    T2 = chain.forward_kinematics(q_ik)
    pos_err = np.linalg.norm(T2[:3, 3] - pos)
    rot_err = np.linalg.norm(T2[:3, :3] - rot)
    print(f"IK solve time: {dt:.1f} ms")
    print(f"IK joints: {np.round([q_ik[i] for i in active_idx], 4)}")
    print(f"True     : {test_vals}")
    print(f"Position error: {pos_err*1000:.3f} mm, rotation error (fro): {rot_err:.5f}")

    ok = pos_err < 0.002 and rot_err < 0.01 and len(joint_names) == 6
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


def test_ik_roundtrip():
    """pytest entry point: run IK round-trip for every installed HC10DTP support package."""
    import pytest
    results = {}
    for pkg in ('hc10dtp_b00_support', 'motoman_hc10_support'):
        try:
            get_package_share_directory(pkg)
        except Exception:
            continue
        results[pkg] = run_roundtrip(pkg)
    if not results:
        pytest.skip("No HC10DTP support package installed")
    assert all(results.values()), f"IK round-trip failed: {results}"


def main():
    results = {}
    for pkg in ('hc10dtp_b00_support', 'motoman_hc10_support'):
        try:
            get_package_share_directory(pkg)
        except Exception:
            print(f"\n=== {pkg} === not installed, skipped")
            continue
        results[pkg] = run_roundtrip(pkg)

    if not results:
        print("\nNo HC10DTP support package installed — nothing tested.")
        sys.exit(1)
    print(f"\nOVERALL: {'PASS' if all(results.values()) else 'FAIL'} ({results})")
    sys.exit(0 if all(results.values()) else 1)


if __name__ == '__main__':
    main()
