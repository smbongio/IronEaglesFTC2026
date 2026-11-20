# FTC 2026 Biobuzz Challenge
**Reference:** [FIRST Tech Challenge Game and Season](https://www.firstinspires.org/programs/ftc/game-and-season)

## Hardware Initialization
In the initialization phase, the robot must identify all hardware. The hardware is as follows:

* **4 wheel motors** (REV DC motor)
* **1 intake motor** (REV DC motor, open-loop power, no encoder)
* **2 shooter motors** (REV DC motors: leader on Expansion Hub port 2 with speed control in ticks/second from its own encoder; follower on port 3 gets the leader's power every loop)
* **1 hopper servo** (REV Smart Servo in continuous mode on Expansion Hub servo port 0, open-loop speed, no position feedback)
* **3 dead-wheel odometry pods** (left and right front-to-back, one side-to-side at the rear)
* **An LED strip** (solid alliance color)

An expansion hub connects intake (port 0), shooter (port 2), and shooter follower (port 3). Odometry pods use Expansion Hub encoder jacks: RIGHT on port 0, LEFT on port 1, REAR (the X wheel) on port 3. The intake shares port 0 with the RIGHT pod and the follower shares port 3 with the REAR pod; both run on power only. Port 1 has no motor. Config names are the hub ports (`chPort0`, `ehPort2`, `ehServo0`, ...); see `HARDWARE.md`. Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`.

> **Note:** Due to the way the wheels are mounted, the **FRONT RIGHT** wheel motor must turn in the inverse direction as all other wheels.

If any single piece of hardware cannot be found, it should not result in an error. The device should report it as a warning and complete initialization.

## Starting Position (this configuration)

The BIOBUZZ field is rotationally symmetric: blue is red rotated 180° about field center (X and Y negated, heading + 180). All field numbers in `TunableValues.java` are red; `config/Alliance.java` converts them. The usual red start is (63, −11, 0°) on the audience wall, so blue's is (−63, +11, 180°) on the far wall. The blue driver stands at the blue wall, so their stick-up is field −Y (red's is +Y).

Live constants (red side): `FIELD_SIZE_IN`, `START_POSE_X_IN`, `START_POSE_Y_IN`, `START_HEADING_DEG`, `START_FAR_POSE_X_IN`, `START_FAR_HEADING_DEG`, `DRIVE_STICK_ZERO_DEG`. Gamepad 1 Back restores the heading TeleOp started with.

### Autonomous

Four programs: **Red Cell Up**, **Red Cell Down**, **Blue Cell Up**, **Blue Cell Down**. At the start, red's audience-side cell and blue's far-side cell are tipped up. The shooter and camera are on the back; the intake bumper touches the start wall.

* **Cell up:** start with the intake bumper on the wall facing the up cell (red: audience wall at (63, −11); blue: far wall at (−63, +11)). Back up 12 in toward the cell (while turning the hopper 90° from its loaded horizontal position to vertical), spin the shooter 2 s, turn the hopper 180° twice (it ends vertical), park.
* **Cell down:** start on the opposite wall (red: (−63, −11); blue: (63, +11)), back up 12 in, wait 10 s for the partner to tip the hive, then shoot and park the same way.
* **Park:** drive along the park lane (Y = −40 for red) to the loading zone (red tile A5, X −47.5 to −24.5 on the red wall; blue tile F2, mirrored by the rotation). The robot center goes on the zone edge nearest the wall it started on, so half the robot is in and the partner has the other half. Intake bumper toward the alliance wall; it pushes into the wall to reset Y and heading.
* **Camera check:** the camera faces the hive. If it sees cell tags in enough frames, it corrects X (only), and re-parks if X is more than 4 in off. If it sees nothing, the robot stays put.
* The Driver Hub shows **CELL SEEN AT** in INIT and during the camera check: where the camera puts each cell it sees if the odometry pose is right. Use it at a known pose to set `VISION_UP_CELL_X_IN`, `VISION_DOWN_CELL_X_IN`, `VISION_CELL_Y_IN`, and measure `CAMERA_*`.

### Alliance selection

1. **Autonomous:** the program records the alliance and its final pose, and preselects the TeleOp.
2. **TeleOp INIT:** picks up both from Autonomous (they last until the Robot Controller app restarts). Gamepad 1 **B** = RED, **X** = BLUE overrides the alliance; **Y** discards Auto's pose and uses the alliance start pose. Without Autonomous, it keeps the last TeleOp's alliance; if none was ever chosen, the Driver Hub shows NOT SET, and pressing start without choosing defaults to RED with a log warning.
3. **Blinkin** shows solid red or blue for the whole match (off while not set).

### Field frame

Official FTC Field Coordinate System ([ftc-docs](https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html)). BIOBUZZ is a square field with the red alliance on the audience-left, same as DECODE inverted.

* **Origin:** center of the field (where the four center tiles meet), on the tile surface
* Field interior is **144 in × 144 in**, so each wall is **±72 in** from the origin
* Stand at the **Red Wall** looking into the field
* **+Y:** away from red, toward the blue alliance (red is min Y, blue is max Y)
* **+X:** to the right, toward the audience (audience wall is max X, far wall is min X)
* **Heading 0°:** +X (toward the audience). **Heading 90°:** +Y (toward blue). **Heading 180°:** into the field

### Required placement

* Robot is **18 in × 18 in** bumper-to-bumper
* **Front bumper** against the audience wall (the +X wall / cell)
* **Near (red-side) bumper** is **52 in** from the red-alliance wall
* **Front** faces +X (toward the audience)

That puts the chassis center at:

| | Value |
|---|---|
| **X** | **+63.0 in** (72 in to the audience wall − 9 in to center) |
| **Y** | **-11.0 in** (52 in to the near bumper + 9 in to center − 72 in to field center) |
| **Heading** | **0°** |

The robot **must** start in this pose for this configuration. A different wall, alliance, or facing needs its own start constants. Driver Hub **X** / **Y** are these field-center inches.

## Driver Input
The robot must take inputs from Gamepad 1 and Gamepad 2.

### Gamepad 1 (Movement)
Gamepad 1 should handle the movement of the robot. 
* The robot will take inputs from the left and right analog sticks to determine the movement of the robot.
* The robot must support "field-centric" motion, meaning the analog stick must respond to move the robot in relation to the field, not the robot’s current direction.

### Gamepad 2 (Operations)
Gamepad 2 should handle the operations of the robot.
* **Right Trigger:** Spin up the shooter, wait until the shooter is at full speed, and then feed the hopper (move 180 degrees).
* **Right Bumper:** Perform an "aim-assist" shot, defined later.
* **Left Trigger:** Spin the intake motor at a predefined speed.
* **Left Bumper:** Spin the intake motor in the opposite direction at full speed.
* **X Button:** Spin the hopper in the feeding direction (move 180 degrees).
* **Y Button:** Spin the hopper in the opposite feeding direction (move -180 degrees).
* **A Button:** Spin the shooter motor in the shooting direction.
* **B Button:** Spin the shooter motor in the opposite shooting direction.

## Vision Control
* **Camera position fixes (Auto and TeleOp):** the camera scans all match. Any of the four cells' tag clusters within 72 in gives a robot X/Y (using the odometry heading). At most every 3 s, odometry X/Y moves to the median of the last second's sightings. Heading is never changed by the camera. Sightings are ignored while turning faster than 30°/s or when more than 12 in from odometry. The Driver Hub shows **CAMERA FIX**; the log shows `[CAMERA FIX]`. Turn off with `VISION_FIX_ENABLED`.
* The LED stays solid alliance color all match (team decision 11/17); in-range feedback, if added, goes on the Driver Hub instead.
* The robot is in range if it is within a 45-degree cone of the hive.
* When locating the hive that is in range, we need to identify the hive that is in our color and is in the upright position.

## Aim Assist Functionality
When the operator hits the "aim assist" button, the following should occur:

1. The robot should identify whether or not it is in a 45-degree cone of the target hive. If it is not within a 45-degree cone of the target hive, we do not enable "aim assist".
2. The robot should begin spinning the shooter so that it is ready to shoot after it is aligned to the target hive.
3. The robot should determine the geometry of the target hive (distance, height, and yaw).
4. The robot should calculate the angle it should rotate in order to align the shooter with the target hive.
5. The robot should move so that it is at the calculated angle. The robot should use the dead-wheel odometry heading in order to perform this movement.
6. The robot should verify it is aligned with the target hive. If it is not, it should go back to the geometry step.
7. The robot should feed the shooter until manual input is received (move the hopper 180 degrees).

## Target Hive Identification
The target hive can be defined by the hive that is in the upright position and is the color of your team. 

**Blue Alliance Targets:**
* The blue hive directly facing the flower has the following IDs: **38, 39, 40, 41**
* The opposite blue hive has the following IDs: **42, 43, 44, 45**

**Red Alliance Targets:**
* The red hive directly facing the flower has the following IDs: **30, 31, 32, 33**
* The opposite red hive has the following IDs: **34, 35, 36, 37**