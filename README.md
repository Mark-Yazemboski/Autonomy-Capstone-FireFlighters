# 🔥 Autonomous Fire Fighting Drone

## A functional, affordable, and easily buildable drone capable of searching an area, detecting a heat source, and deploying a payload.  
🏆 **First Place Winner – Senior Capstone Showcase**

This senior capstone project's objective was to design and build a drone capable of autonomously searching a designated area, locating a simulated fire using a thermal camera, and deploying a fire suppressant. Each team was given only the following components: an **Arduino**, a **thermal camera**, a **transmitter/receiver**, and an **IMU**.

> 👨‍🚒 **Team Name:** *The FireFlighters*

---

### 📸 Team Photo

![Fire Fighting Drone](images/Team.jpg)

**Team Members (left to right):**

- **Achutan Srinivasan** – Back Left  
- **Ankur Ghorai** – Back Middle Left  
- **Jaisel Singh** – Back Middle Right  
- **Trusha Patel** – Back Right  
- **Mark Yazemboski** (👋 Me, Front) – *Team Lead*

---

### 🛠️ Key Design Principles

Our team focused on four core physical design goals:

- ✅ Fully 3D-printed frame  
- 🛒 Easily obtainable parts from Amazon and other fast-shipping retailers  
- 🧩 Modular, repairable design  
- ⚡ Clean and efficient wire management  

---

### 💡 Design Rationale

We studied the successes and failures of previous teams to refine our own strategy. Our key choices included:

- A **fully 3D-printed frame** to enable rapid repairs and keep material costs low.
- Using fast-shipping vendors like **Amazon** and **GetFPV** to avoid long lead times—even if the parts cost slightly more.
- A **modular design**, where every part was independently printable and easily replaceable. For example, a broken arm could be replaced in hours, not days.
- Emphasis on **wire management**, influencing how we designed the baseplate and arm connections. ESC wiring was routed cleanly through the frame for safety and reliability.

---

### 🧠 Drone Architecture (CAD Model)

![CAD Model](images/CAD_Drone.png)

A more detailed breakdown is available in our final report. Here's a high-level look at the three-tier frame design:

1. **Base Plate**  
   - The structural foundation of the drone  
   - Connects the arms, battery, payload mechanism, and power distribution board (PDB)  
   - Serves as the central hub of the entire system

2. **Top Plate**  
   - Houses the Arduino and IMU hat  
   - Elevated to provide space and reduce electromagnetic interference from high-current motor wires

3. **Shield**  
   - Supports motion capture (MoCap) balls used for position and orientation tracking  
   - Protects sensitive electronics like the Arduino in case of a crash or flip

---

### ✈️ Arm Design

The arms were custom-designed as **C-beams with trusses**:

- The **C-beam profile** allowed us to route ESCs beneath the arm cleanly and reduced deflection during flight.
- **Trusses** helped save weight while maintaining stiffness, which was especially important given the frame’s 3D-printed nature.
- These design choices improved the structural integrity and flight stability of the drone.

---

### 🖨️ 3D Printing and Cost Efficiency

![3D Print Layout](images/3D_Print.png)

All drone parts were printed on a **Prusa XL** using PLA. The full frame:

- Took approximately **13 hours** to print  
- Used just **$3.38** worth of filament  
- Demonstrated how functional UAV platforms can be both low-cost and rapidly manufacturable

---

### 📄 Final Report

For more technical details, component specs, control logic, and results from our flight tests, please refer to the **Final Report** included in this repository.
