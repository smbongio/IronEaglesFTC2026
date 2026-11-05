# FTC 2026 Biobuzz Challenge
**Reference:** [FIRST Tech Challenge Game and Season](https://www.firstinspires.org/programs/ftc/game-and-season)

## Hardware Initialization
In the initialization phase, the robot must identify all hardware. The hardware is as follows:

* **4 wheel motors** (REV DC motor)
* **1 intake motor** (REV DC motor, spins rapidly to intake balls into the robot)
* **2 shooter motors** (REV DC motors, geared to the shooter wheel): `shooter` (leader) and `shooterFollow` (copies the leader's power each command). There is no hub follow-mode; the code sends the same command to both.
* **1 hopper motor** (REV Hex Core motor, spins at a pre-defined speed to feed the balls into the shooter)
* **An LED strip** (used for aim assist)

An expansion hub will be used to connect the intake motor, shooter motors, and hopper motor. Add `shooterFollow` on the Driver Station (Expansion Hub port 3 in `IronEaglesConfig.xml`). Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`. If the follow motor spins the wrong way, flip `SHOOTER_FOLLOW_DIR`.

> **Note:** Due to the way the wheels are mounted, the **FRONT RIGHT** wheel motor must turn in the inverse direction as all other wheels.

If any single piece of hardware cannot be found, it should not result in an error. The device should report it as a warning and complete initialization.

## Starting Position (this configuration)

Field-centric drive and OctoQuad pose assume **one** start: red alliance, audience wall, robot upright facing into the field.

Live constants: `FIELD_SIZE_IN`, `START_POSE_X_IN`, `START_POSE_Y_IN`, `START_HEADING_DEG` in `TunableValues.java`. Gamepad 1 Back restores this heading.

### Field frame

* **Origin:** center of the field (where the four center tiles meet), on the tile surface
* Field interior is **144 in × 144 in**, so each wall is **72 in** from the origin
* **+X:** toward the blue alliance, along the audience wall
* **+Y:** into the field (away from the audience)
* **Heading 0°:** +X (toward blue). **Heading 90°:** +Y (into the field)
* Negative X is toward red. Negative Y is toward the audience.

### Required placement

* Robot is **18 in × 18 in** bumper-to-bumper
* **Back bumper** against the audience wall (shooter / back faces the audience cell)
* **Near (red-side) bumper** is **52 in** from the red-alliance wall
* **Front** faces into the field (+Y)

That puts the chassis center at:

| | Value |
|---|---|
| **X** | **-11.0 in** (52 in to the near bumper + 9 in to center − 72 in to field center) |
| **Y** | **-63.0 in** (9 in from the audience wall − 72 in to field center) |
| **Heading** | **90°** |

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
* The LED should flash quickly when the robot is in range.
* The robot is in range if it is within a 45-degree cone of the hive.
* When locating the hive that is in range, we need to identify the hive that is in our color and is in the upright position.

## Aim Assist Functionality
When the operator hits the "aim assist" button, the following should occur:

1. The robot should identify whether or not it is in a 45-degree cone of the target hive. If it is not within a 45-degree cone of the target hive, we do not enable "aim assist".
2. The robot should begin spinning the shooter so that it is ready to shoot after it is aligned to the target hive.
3. The robot should determine the geometry of the target hive (distance, height, and yaw).
4. The robot should calculate the angle it should rotate in order to align the shooter with the target hive.
5. The robot should move so that it is at the calculated angle. The robot should use the IMU system in order to perform this movement.
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