# Software Requirements Specification: FTC TeleOp 2026

## 1. Overview
This document outlines the software requirements and acceptance criteria for a Java-based TeleOp program controlling a custom 4-motor Mecanum drivetrain robot. The project uses a local development workflow via Android Studio, targeting a REV Control Hub and Expansion Hub architecture. 

## 2. Hardware Configuration
The software must map to the following active hardware configuration on the REV hubs.

### 2.1 Control Hub (Main)
*   **Motor Port 0:** `frontRight`
*   **Motor Port 1:** `backRight`
*   **Motor Port 2:** `backLeft`
*   **Motor Port 3:** `frontLeft`
*   **USB:** `camera` (Webcam)

### 2.2 Expansion Hub
*   **Motor Port 0:** `intake`
*   **Motor Port 1:** `hopper`
*   **Motor Port 2:** `shooter`

## 3. Control Scheme (Logitech F310 Gamepad)
The robot is operated by a single driver (Gamepad 1) utilizing a momentary (hold-to-run) control paradigm for all mechanisms to ensure zero input lag and eliminate state-tracking bugs.

| Input | Sub-System | Action |
| :--- | :--- | :--- |
| **Left Analog Stick (X/Y)** | Drivetrain | Forward, Backward, Strafe Left, Strafe Right |
| **Right Analog Stick (X)** | Drivetrain | Rotate Left, Rotate Right |
| **X Button** | Intake | Hold to run intake motor at preset speed |
| **Left Trigger (L2)** | Hopper | Hold past 20% deadzone to run hopper motor |
| **Right Trigger (R2)** | Shooter | Hold past 20% deadzone to run shooter motor |

## 4. Software Architecture & Logic Constraints

### 4.1 Abstraction of Motor Directions
To facilitate rapid field adjustments without modifying core logic, all motor directions must be abstracted using static multiplier constants at the top of the OpMode class.
*   **Implementation:** Target speeds are multiplied by their respective direction constant (`1` or `-1`) before being written to the motor hardware.
*   **Variables Required:** `FRONT_LEFT_DIR`, `FRONT_RIGHT_DIR`, `BACK_LEFT_DIR`, `BACK_RIGHT_DIR`, `INTAKE_DIR`, `HOPPER_DIR`, `SHOOTER_DIR`.

### 4.2 Drivetrain Kinematics
*   The software must implement standard Mecanum kinematic equations to convert the three analog joystick axes into four independent wheel powers.
*   The Left Stick Y-axis input must be inverted in software, as the FTC SDK natively registers "forward" joystick movement as a negative value.

### 4.3 Trigger Thresholds
*   Analog triggers (L2/R2) report values from `0.0` to `1.0`. The software must apply a `> 0.2` threshold to prevent accidental actuation from resting fingers.

## 5. Acceptance Criteria

*   **AC1: Compilation & Deployment:** The Java program compiles successfully in Android Studio and deploys to the REV Control Hub without syntax or SDK dependency errors.
*   **AC2: Hardware Initialization:** The OpMode initializes on the Driver Hub without throwing "hardware devices not found" exceptions for any of the 8 defined peripherals.
*   **AC3: Direction Reversal:** Modifying a direction constant from `1` to `-1` successfully reverses the corresponding motor's spin direction without requiring changes to the core drive or mechanism logic blocks.
*   **AC4: Omnidirectional Drive:** Pushing the left stick forward/backward/left/right moves the chassis translationally, and pushing the right stick left/right rotates the chassis on its center axis.
*   **AC5: Momentary Mechanisms:** The intake, hopper, and shooter motors actuate immediately when their respective buttons/triggers are held, and immediately stop (`power = 0.0`) when released.