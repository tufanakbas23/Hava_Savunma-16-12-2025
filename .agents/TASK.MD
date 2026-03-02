# Project Tasks and Phasing (tasks.md)

**Project:** Autonomous Object Tracking and Alignment System - Computer Vision Pipeline
**Architecture:** Python (Vision, ZED SDK, OpenCV) -> UART Communication -> Arduino C++ (PID, Pan-Tilt Control)

This project is divided into 4 main Stages and subsequent development phases. Standard autonomous system terminology (detection, tracking, alignment, lock-on, actuator) will be used throughout the development process.

## STAGE 1: Static Object Detection and UI Integration (Phase 1)
In this stage, the system operates in manual alignment mode. The primary goal of the vision module is to provide rich situational awareness to the user via the UI.

* **Phase 1.1: Model Integration & Classification**
  * Capture RGB video stream from the ZED 2 camera into the Python environment.
  * Integrate the YOLO model to perform real-time detection of 4 main object classes (Class_1, Class_2, Class_3, Class_4).
* **Phase 1.2: Rough Depth Estimation**
  * Utilize the ZED SDK to calculate the approximate distance of the detected objects using the center points of their bounding boxes.
* **Phase 1.3: UI Data Feeding**
  * Draw the Class Name, Confidence Score, and ZED-derived Distance on the targets.
  * Stream this annotated data to the User Interface (UI) to assist manual lock-on sequences.

## STAGE 2: Multi-Object Tracking & Trajectory Analysis (Phase 2)
The system transitions to autonomous mode. Dynamic objects will approach the system along predefined trajectories.

* **Phase 2.1: Multi-Object Tracking**
  * Integrate a tracking algorithm (e.g., DeepSORT or ByteTrack) on top of the YOLO detections.
  * Assign unique IDs to moving objects to ensure continuous tracking across frames.
* **Phase 2.2: Vectorial Direction & Approach Analysis**
  * Analyze the motion vector in the X-Y plane and the depth variation (Z-axis) of the tracked objects.
  * Prioritize objects that are *approaching* the system and filter out *receding* objects (non-threats).
* **Phase 2.3: Autonomous Lock-on & UART Data Transfer**
  * Determine the primary target and calculate its pixel error coordinates (Error_X, Error_Y) relative to the camera center.
  * Transmit these coordinates to the Arduino via UART to feed the PID loops.

## STAGE 3: Color/Shape Filtering and Dynamic Range Validation (Phase 3)
This is the advanced autonomous stage where the decision mechanism relies on visual markers and depth thresholds.

* **Phase 3.1: Target Discrimination (Color/Shape Filter)**
  * Apply OpenCV Color Thresholding within the Region of Interest (ROI) of the YOLO bounding box.
  * Retain objects that possess the designated color/marker (e.g., specific colored balloons) in the tracking list, and filter out the rest.
* **Phase 3.2: Class-Specific Dynamic Range Masking**
  * Dynamically validate the operation range based on the object's class on the Python side (e.g., Class_1 requires 10-15m, Class_2 requires 5-15m).
* **Phase 3.3: 1D LiDAR Final Verification**
  * Once the pan-tilt motion stabilizes and the camera is locked (PID error nears zero), evaluate the precise distance data from the 1D LiDAR.
  * Generate the final execution signal if the LiDAR distance matches the class-specific range criteria.

## STAGE 4: Parallax Compensation & Dynamic Axis Alignment (Phase 4)
This stage resolves the physical spatial offset between the visual sensor (moving camera) and the physical actuator (alignment mechanism).

* **Phase 4.1: Physical Offset Definition**
  * Define the physical gap (e.g., 10 cm horizontal offset) between the camera and the actuator as a static system parameter.
* **Phase 4.2: Depth-Dependent Dynamic Pixel Shift Calculation**
  * Formulate an algorithm that converts this physical 10 cm offset into a dynamic pixel value based on the real-time distance provided by the ZED or LiDAR.
  * Calculate a "virtual center point" by shifting the target coordinates (left/right) by this calculated pixel value, rather than aiming at the dead center of the camera frame.
* **Phase 4.3: Actuator-Centric PID Setpoint Update**
  * Transmit these dynamically shifted pixel targets to the Arduino via UART.
  * Ensure the PID algorithm aligns the *actuator* to the target, compensating for the camera's physical offset.