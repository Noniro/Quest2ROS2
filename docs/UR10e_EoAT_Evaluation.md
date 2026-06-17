# Business Case & Technical Evaluation: End-of-Arm Tooling for UR10e

This document outlines the justification for selecting the **Robotiq 2F-85 / 2F-140 Adaptive Gripper** series as the core end-effector for our Universal Robots UR10e cobot, alongside our modular vision integration options.

---

## 1. Why Select the Robotiq 2F-85 / 2F-140?

The Robotiq 2F series is the global industry standard for collaborative robotic manipulation. Implementing this gripper on the UR10e provides several key advantages:

### A. Dual Gripping Modes (Parallel & Enveloping)
* **Adaptive Finger Design:** The unique linkage system allows the gripper to automatically switch between an internal/external parallel grip and an enveloping (enclosing) grip depending on the geometry of the object.
* **Versatility:** Eliminates the need to change tools or design custom pneumatic fingers for every unique part shape.

### B. Seamless UR integration (UR+ Certified)
* **Plug-and-Play software:** Fully compatible via **URCaps**. The gripper setup, stroke control, and force adjustments are integrated directly into the UR PolyScope teach pendant software.
* **Fast Implementation:** Reduces engineering and deployment time from weeks to hours compared to generic industrial grippers that require custom PLC logic or external I/O mapping.

### C. Precision Control
* **Adjustable Stroke & Force:** * **2F-85:** Offers a stroke of up to 85mm.
  * **2F-140:** Offers a wider stroke of up to 140mm for larger parts.
  * Both units feature programmable grip force (20 to 235 N) and speed, allowing safe handling of both fragile items and heavy components up to the UR10e's capacity.

---

## 2. Vision Integration & Camera Mounting Options

To achieve an autonomous eye-in-hand setup without knowing our final sensor requirements today, we have narrowed down our integration path to two distinct options:

### Option A: Custom CNC-Machined Adapter Plate (Maximum Flexibility)
We can design an independent "sandwich" extension plate that bolts between the UR10e wrist flange and the Robotiq gripper. This plate will be prototyped via 3D printing and finalized by machining it out of **6061-T6 Aluminum using a CNC machine**.

* **Pros:** * Total hardware modularity. We are not locked into any single camera vendor.
  * Allows us to swap, test, and upgrade to various 2D or 3D depth cameras (e.g., Intel RealSense, Orbbec, Lucid) at any time.
  * Low cost of hardware fabrication.
* **Cons:** Requires internal engineering time for mechanical design, cable routing, and hand-eye calibration scripts.

### Option B: Robotiq Dedicated Wrist Camera (Maximum Speed & Integration)
We can purchase Robotiq’s official **Wrist Camera**, which is designed specifically to act as a mechanical spacer between the UR flange and the 2F gripper.

* **Pros:**
  * Cleanest physical integration: Zero design effort required; fits the ISO 9409-1 bolt pattern perfectly on both sides.
  * Software-defined vision: Works natively within the UR teach pendant, providing simple tools for visual picking, object teaching, and automatic part orientation tracking.
  * Sealed housing built for industrial environments.
* **Cons:** High upfront cost and binds us strictly to the resolution and feature set of the Robotiq vision ecosystem.

---

## 3. Recommendation Summary

For immediate project velocity, we recommend acquiring the **Robotiq 2F-85** (or 2F-140 if parts exceed 85mm in width). 

While deciding on the definitive camera architecture, we should proceed with **Option A (Designing a custom CNC-ready mount)**. This path keeps our hardware options wide open and allows us to benchmark various vision systems at a fraction of the cost before committing to a final production configuration.
