# FTC 2026 Biobuzz Challenge
**Reference:** [FIRST Tech Challenge Game and Season](https://www.firstinspires.org/programs/ftc/game-and-season)

## How to Read This Document
Each section is split into two parts:

* **Requirement:** what the robot must do, as seen by the drivers. These change rarely, and every requirement should be testable on the field.
* **Design:** how we have decided to meet the requirement right now. These can change as we test, as long as the requirement is still met.

Facts we don't control, such as the field layout, the AprilTag IDs, and how our robot is built, are in the **Reference** section at the end. Every tunable value is listed in the **Tunable Values** table at the end.

---

## Hardware and Initialization

### Requirement
* During initialization, the robot must look for every device listed in Reference: Robot Hardware.
* If any single device cannot be found, it must not cause an error. The robot reports it as a warning on the Driver Hub and completes initialization.
* With a device missing, the robot keeps as much working as it can:
  * **No webcam:** auto fire and the auto-fire availability notification are disabled. Everything else works normally.
  * **No IMU:** driving falls back to robot-centric (the sticks move the robot relative to the way it is facing). Auto fire still works, because it navigates using the camera.

### Design
* The webcam is named `camera` in the robot configuration.
* The IMU is named `imu` in the robot configuration. The Driver Hub adds this entry automatically (on I2C port 0), and it must not be deleted or renamed.
* The code must be told how the Control Hub is mounted (see Reference: Robot Hardware) so that the IMU measures the robot's heading correctly.

---

## Alliance Selection

### Requirement
The driver chooses the alliance (Red or Blue) during initialization, after pressing INIT and before pressing Play.

* On **Gamepad 1**, press **X for Blue** or **B for Red**.
* The choice can be changed any number of times before Play. The last press wins.
* The Driver Hub shows the current choice, or **NOT SET** if no alliance has been chosen.
* There is no default alliance. If Play is pressed without choosing one:
  * Auto fire is disabled (the Right Trigger does nothing), and the Driver Hub keeps showing a warning that the alliance is not set.
  * Everything else (driving, intake, hopper, shooter) works normally.
  * The driver can still choose an alliance with the same buttons. Once chosen, auto fire works and the alliance is locked for the rest of the match.
* Once Play is pressed with an alliance chosen, the alliance cannot be changed for the rest of the match.

### Design
The SDK supports this directly: `opModeInInit()` keeps a loop running until Play is pressed, and gamepad methods such as `xWasPressed()` detect a single button press.

---

## Shooter

### Requirement
* The robot must be able to shoot at any time during TeleOp, with no wait for the shooter to spin up.
* Shot strength must stay consistent for the whole match, including as the battery drains.
* There are no driver controls for the shooter.

### Design
The shooter runs continuously for the entire match at one constant speed.

* The shooter starts spinning when the driver presses Play, not during initialization.
* Nothing in normal operation stops the shooter, including cancelling auto fire. It stops only when the match ends.
* The speed is a target speed in **RPM**, set by one tunable value, `SHOOTER_TARGET_RPM`, found by testing.

The shooter is controlled by target speed, not by motor power. With a fixed power, the shooter slows down as the battery drains over the match, and shots fall short. With a target speed, the motor controller on the hub adjusts the power on its own to hold the RPM. This also brings the wheel back up to speed quickly after each shot.

This uses the FTC SDK's built-in velocity control on `DcMotorEx`:

```java
shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

double ticksPerRev = shooter.getMotorType().getTicksPerRev();
shooter.setVelocity(SHOOTER_TARGET_RPM / 60.0 * ticksPerRev); // ticks per second
```

* The shooter motor's **encoder cable must be connected** to the expansion hub. Without it, velocity control does not work.
* The shooter motor must be set to the correct motor type in the robot configuration on the Driver Station. `getTicksPerRev()` depends on it, so the wrong motor type gives the wrong speed.
* The RPM is measured at the motor's output shaft. If there is gearing or a belt between the motor and the shooter wheel, the wheel spins at a different speed.

> **Note:** Running the shooter all match still needs to be tested. The main risk is battery drain, which we don't expect to be a problem.

---

## Driving (Gamepad 1)

### Requirement
Driving is **field-centric**: the sticks move the robot relative to the driver's view of the field, not relative to the way the robot is facing.

* **Left stick:** move the robot, from the driver's point of view.
  * Up: away from the driver.
  * Down: toward the driver.
  * Left and right: to the driver's left and right.
* **Right stick (left/right):** turn the robot in place.
* **Back button** (called Share on PlayStation-style gamepads): reset forward. The driver points the robot straight away from themselves and presses it, and that direction becomes the new forward. This corrects drift during the match, or a wrong starting direction.

**Drive team procedure:** before pressing INIT, place the robot **facing straight away from the driver** (for example, with its back against our own alliance's wall).

### Design
The red and blue drivers stand on opposite sides of the field, facing each other (see Reference: Field), so "away from the driver" points in opposite directions for the two alliances.

The IMU only measures how far the robot has turned since it was last zeroed, so forward is set like this:
* The IMU heading is zeroed during initialization, while the robot is facing away from the driver. From then on, forward means away from the driver, for either alliance.
* Because forward is set from the robot's starting direction, driving does not depend on the alliance selection.
* The Back button zeroes the IMU heading again.

> **Note:** If an autonomous program runs before TeleOp, the robot will not be facing away from the driver when TeleOp starts, and it cannot be touched during the transition. In that case the driver uses the Back button to reset forward once TeleOp begins. Passing the robot's heading from autonomous to TeleOp is out of scope for now.

---

## Operator Controls (Gamepad 2)

### Requirement
* **Right Trigger:** start auto fire (see Auto Fire). This is the only way to shoot.
* **Left Trigger:** while held, run the intake inward at a predefined speed. Stops when released.
* **Left Bumper:** while held, run the intake outward at a predefined reverse speed. Stops when released.
* **X Button:** step the hopper 180 degrees in the feeding direction.
* **Y Button:** step the hopper 180 degrees in the opposite direction (-180 degrees).

The triggers are either held or not. How far a trigger is pulled does not change anything. A small deadzone decides when a trigger counts as held.

For both hopper buttons:
* Pressing the button starts one 180-degree step.
* When the step finishes, the robot checks whether the button is still held. If it is, the hopper does another 180-degree step, and so on until the button is released.
* A step always finishes once started, even if the button is released partway through. This keeps the hopper lined up at a 180-degree position.

### Design
Both intake speeds are target speeds in **RPM**, each set by its own tunable value:
* `INTAKE_TARGET_RPM`: the inward speed, used by the Left Trigger.
* `INTAKE_REVERSE_RPM`: the outward speed, used only by the Left Bumper.

They use the same SDK velocity control as the shooter:

```java
intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

double ticksPerRev = intake.getMotorType().getTicksPerRev();
intake.setVelocity(INTAKE_TARGET_RPM / 60.0 * ticksPerRev);    // Left Trigger: inward, ticks per second
intake.setVelocity(-INTAKE_REVERSE_RPM / 60.0 * ticksPerRev);  // Left Bumper: outward, ticks per second
intake.setVelocity(0);                                         // released: stop
```

* As with the shooter, the intake motor's **encoder cable must be connected** to the expansion hub, and the motor must be set to the correct motor type in the robot configuration.

> **Note:** It is not yet confirmed whether the intake motor's encoder cable is connected. If it is not, velocity control will not work, and the intake will need to use fixed power values instead of RPM.

---

## Auto Fire Availability Notification

### Requirement
* The operator must be notified when **auto fire is available**: an alliance has been chosen and the camera can see the target cell's tags. Pulling the Right Trigger at that moment starts auto fire.
* The notification must be obvious during a match without the operator having to look at the Driver Hub.

### Design
* The LED strip flashes quickly while auto fire is available.

---

## Auto Fire

### Requirement
Auto fire is a one-press action. A single pull of the Right Trigger on Gamepad 2 starts the whole sequence: the robot drives itself to a shooting position, aims at the target cell, and keeps shooting until a driver takes over. It doesn't matter whether the trigger is held down or released after the pull.

The shooter runs at a constant RPM (see Shooter), so shooting from the same distance every time gives a consistent shot. That is why auto fire moves the robot to a fixed distance from the target cell, not just turns it.

* **Starting:** auto fire starts if the camera can see the target cell's tags when the trigger is pulled, wherever the robot is. If it can't, auto fire does not start and nothing else happens.
* **Moving:** the robot drives itself to the **closest point on the shooting arc** (defined below) and aims the shooter at the target cell. It confirms that it is at the right distance and still aimed before shooting.
* **Shooting:** the robot shoots repeatedly, one ball at a time, until auto fire ends.
* **Manual input ends auto fire.** Manual input is any button press, trigger pull, or joystick movement on **either Gamepad 1 or Gamepad 2**, except the Right Trigger on Gamepad 2. A small deadzone applies to joysticks and triggers so stick drift doesn't count as input. Manual input ends auto fire at any point in the sequence.
* **Losing the target:** the robot must not keep driving while it can't see the target cell. Short interruptions of the camera's view (for example, another robot driving past) must not end auto fire. If the target cell's tags stay out of view for longer than a set time, auto fire ends.
* **When auto fire ends**, for any reason: the robot stops moving, the hopper stops, the shooter keeps spinning, and normal control returns to the drivers.
* **Pulling the trigger again** while auto fire is running does nothing. It neither restarts nor cancels the sequence.
* Auto fire does not try to detect when the HIVE tips. It keeps shooting until manual input, unless the target goes out of view as described above.
* **Obstacles:** auto fire does not avoid obstacles such as the HIVE frame, the downward-facing CELLS, or other robots. The drivers are responsible for watching the robot's path and cancelling auto fire with any manual input if it is about to hit something.

**Shooting cone:** the area of the field from which the robot is allowed to shoot at the target cell.

* Picture a triangle lying flat on the field. Its point is directly below the center of the target cell's AprilTag cluster.
* The center line of the triangle points the way the tag cluster faces. The tags are on the underside of the cell and tilt downward, so this means the direction they face when flattened onto the field, ignoring how far they tilt down.
* Each side of the triangle is **45 degrees** from that center line, so the cone is **90 degrees wide** in total.
* Put another way: standing under the cell and looking the way its tags face, a position is in the cone if it is no more than 45 degrees to the left or right.

The cone is measured from the cell's point of view, not the robot's. It doesn't matter which way the robot is facing or where the cell appears in the camera image.

**Shooting arc:** the set of positions auto fire shoots from. It is the curved line inside the shooting cone that is exactly the **shooting distance** from the point under the target cell's tag cluster. Every point on the arc gives the same shot. The closest point on the arc to the robot is found like this:

* **If the robot is inside the cone:** the point on the arc directly toward or away from the cell. The robot moves straight in or out until it is at the shooting distance.
* **If the robot is outside the cone:** the nearer end of the arc, at the 45-degree edge of the cone.

### Design
Auto fire navigates using the camera alone. The robot faces the target cell and keeps its tags in view the whole way, so every loop gets a fresh reading of where the cell is. Road Runner, path following, and wheel odometry are not used.

Auto fire runs these steps:

1. Check whether the camera can see the target cell's tags. If it can't, auto fire does not start.
2. Measure the target cell's position from the camera: distance, bearing (where the cell is in the camera's view), and yaw (the robot's angle off the cell's center line).
3. Work out the closest point on the shooting arc, and how far the robot is from it.
4. Drive toward that point while turning to face the cell. The mecanum wheels let the robot do all three movements at once, each with its own simple correction:
   * **Forward/back:** proportional to the distance error (current distance minus `SHOOTING_DISTANCE_IN`).
   * **Turn:** proportional to the bearing error, so the robot keeps facing the cell. `AIM_OFFSET_DEG` is added to the bearing to correct for shots that consistently miss to one side during testing. It starts at 0.
   * **Strafe:** only while the robot is outside the cone, proportional to how far past the 45-degree edge it is. Inside the cone, the robot does not strafe.
5. Verify the robot is in position. If it is not, go back to step 2. The robot is in position when all of these are true:
   * the distance is within `DISTANCE_TOLERANCE_IN` of `SHOOTING_DISTANCE_IN`;
   * the aim (bearing, including `AIM_OFFSET_DEG`) is within `ALIGNMENT_TOLERANCE_DEG`, to either side;
   * the robot is inside the shooting cone.
6. Stop the drive motors and feed the shooter by turning the hopper 180 degrees and then stopping. Repeat, one 180-degree step at a time, until auto fire ends.

The SDK sample `RobotAutoDriveToAprilTagOmni` already does most of step 4: it drives to a set distance from a tag while facing it, using the same three corrections. Auto fire differs in one way: the sample always strafes to square up with the tag, while auto fire strafes only until the robot is inside the cone.

The shooter is already spinning for the whole match (see Shooter), so auto fire does not need to start it.

**Shooting distance.** `SHOOTING_DISTANCE_IN` is measured by the camera, which sits about 6 inches behind the shooter (see Reference: Robot Hardware). Because the value is tuned by testing, the offset is absorbed and needs no correction.

**Losing sight of the tags.** One tunable value, `TAG_LOSS_TIMEOUT_SEC` (for example, 1 second), controls how long auto fire waits for the tags to come back:

* **While driving and verifying (steps 2 to 5):** stop moving and wait. If the tags come back within the timeout, continue from step 2. If not, auto fire ends.
* **During feeding (step 6):** keep feeding through short losses. If the tags stay out of view for longer than the timeout, auto fire ends.

> **To be written:** auto fire will be built as a state machine. Its states and the transitions between them are still to be designed.

---

## Target Cell

### Requirement
* **The target cell is the upward-facing CELL of our alliance's color** (see Reference: Field for what a CELL is).
* Only one of our two CELLS can be up at a time, so there is always exactly one target cell.

### Design
* The target cell is found using the AprilTags on the CELLS (see Reference: Field for the IDs).
* All four tags in a cluster are on the same sticker, so any of them gives the cluster's position. When more than one is visible, the code can combine them for a steadier reading.

**Telling which CELL is up.** The upward-facing CELL's tags sit much higher above the tiles than the downward-facing CELL's, so the code uses the height of the tags:

* Every AprilTag reading includes the tag's distance from the camera (`range`) and how far above the camera's center line it appears (`elevation`). From these, plus the camera's own height and upward tilt (see Reference: Robot Hardware), the code calculates the tag's height above the tiles:

  ```
  tag height above tiles = camera height + range × sin(elevation + camera tilt)
  ```

* A CELL of our alliance's color counts as upward-facing if its tag height is above a tunable value, `CELL_UP_MIN_HEIGHT_IN`. This value is set between the heights of an upward-facing and a downward-facing CELL's tags, measured on the field.
* This height is calculated every time the code reads the target cell's geometry.

---

## Reference

### Field
From the BIOBUZZ Competition Manual (Sections 9.5, 9.6, and 9.9). This document uses the manual's terms:

* **HIVE:** one of the two seesaw structures on the frame in the center of the field. There is one red HIVE and one blue HIVE, sitting side by side, parallel to each other.
* **CELL:** one end of a HIVE, which balls are launched into. Each HIVE has two CELLS, one at the audience end and one at the far end, so there are four CELLS on the field.

Each HIVE pivots like a seesaw and is always in one of two positions, with exactly one of its CELLS facing up. Launching enough balls into the upward-facing CELL tips the HIVE, so the other CELL becomes the upward-facing one.

**Drivers:** the drivers stand in their ALLIANCE AREA, on the side of the field: red on the left as seen from the audience, blue on the right. The red and blue drivers face each other across the field.

**AprilTags:**
* Each CELL has one **AprilTag cluster**: four AprilTags in a row on a single sticker.
* The cluster is on the bottom face of the CELL, facing down toward the tiles, with its bottom edge toward the center of the field. The camera has to see the tags from below.
* The tags are from the **36h11** family and are **3.25 in (8.25 cm)** square. The vision code needs both values to measure distance and angle correctly.

| CELL | AprilTag IDs |
|---|---|
| Red, far side (opposite the audience) | **30, 31, 32, 33** |
| Red, audience side | **34, 35, 36, 37** |
| Blue, audience side | **38, 39, 40, 41** |
| Blue, far side (opposite the audience) | **42, 43, 44, 45** |

### Robot Hardware
* **4 wheel motors** (REV DC motor)
* **1 intake motor** (REV DC motor, spins rapidly to intake balls into the robot)
* **1 shooter motor** (REV DC motor, spins rapidly to launch balls into the CELLS)
* **1 hopper motor** (REV Core Hex motor, feeds balls into the shooter)
* **An LED strip**
* **A webcam** (sees the AprilTags on the CELLS)
* **The IMU** built into the Control Hub (measures which way the robot is facing)

An expansion hub connects the intake motor, shooter motor, and hopper motor.

Due to the way the wheels are mounted, the **FRONT RIGHT** wheel motor must turn in the inverse direction as all other wheels.

**Webcam:**
* Plugs into a USB port on the Control Hub.
* Mounted on top of the robot, at the center of the robot, in line with the shooter (no left/right offset).
* The shooter is about 6 inches in front of the camera. Distances measured by the camera are therefore about 6 inches longer than the actual distance from the shooter. Auto fire's shooting distance is measured by the camera and tuned by testing, so no correction is needed.
* The AprilTags face down from the underside of the CELLS, so the camera must be tilted up enough to see them from where the robot shoots.
* **To be filled in:** the camera's height above the tiles and its upward tilt angle. Both are needed to tell which CELL is up (see Target Cell).

**IMU:**
* Built into the Control Hub, so there is no wiring.
* The code needs to know how the Control Hub is mounted, given as two directions: which way the REV logo on the hub faces, and which way the hub's USB ports face (each one of up, down, forward, backward, left, or right).
* **To be filled in:** the Control Hub's logo direction and USB direction on our robot.

---

## Tunable Values
All of these are found or adjusted by testing.

| Value | What it controls | Starting value |
|---|---|---|
| `SHOOTER_TARGET_RPM` | Shooter speed for the whole match | To be found by testing |
| `INTAKE_TARGET_RPM` | Intake inward speed (Left Trigger) | To be found by testing |
| `INTAKE_REVERSE_RPM` | Intake outward speed (Left Bumper) | To be found by testing |
| `SHOOTING_DISTANCE_IN` | Distance from the target cell that auto fire shoots from, as measured by the camera | To be found by testing |
| `DISTANCE_TOLERANCE_IN` | How close to the shooting distance the robot must be before auto fire shoots | To be found by testing |
| `AIM_OFFSET_DEG` | Correction added to the auto-fire aim angle | 0 |
| `ALIGNMENT_TOLERANCE_DEG` | How close the aim must be before auto fire shoots | To be found by testing |
| Auto-fire drive gains and maximum speeds | How hard the forward, strafe, and turn corrections push, and their speed limits | Start from the SDK sample's values |
| `CELL_UP_MIN_HEIGHT_IN` | Tag height above the tiles that counts as an upward-facing CELL | Between the measured up and down heights |
| `TAG_LOSS_TIMEOUT_SEC` | How long auto fire waits for lost tags to come back | About 1 second |
| Deadzone | How far a stick or trigger must move to count as input | To be found by testing |
