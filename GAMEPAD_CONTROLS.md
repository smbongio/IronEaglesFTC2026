# Gamepad controls

TeleOp: `IronEaglesTeleOp2026`.

## Gamepad 1 (driver)

| Input | Action |
|---|---|
| Left stick | Drive, field-centric (stick-up drives away from your alliance wall) |
| Right stick left/right | Turn |
| Back | Reset heading to the TeleOp start heading. After Autonomous that is the park heading (intake facing your alliance wall); otherwise the alliance start heading (intake facing the start wall). Point the robot that way first. |

During **INIT** (before pressing start):

| Input | Action |
|---|---|
| B | Alliance RED |
| X | Alliance BLUE |
| Y | Ignore Autonomous' final pose; start from the alliance start pose |

## Gamepad 2 (operator)

| Input | Action |
|---|---|
| Right trigger (hold) | Shooter forward at the target speed |
| Right bumper (hold) | Shooter reverse at the target speed |
| D-pad up (tap) | Shooter target +50 ticks/s |
| D-pad down (tap) | Shooter target −50 ticks/s |
| Left trigger (hold) | Intake forward (50% power) |
| Left bumper (hold) | Intake reverse (50% power) |
| X (hold) | Hopper forward |
| Y (hold) | Hopper reverse |

The shooter target starts at 1000 ticks/s and stays between 0 and 3000. The Driver Hub's `SHOOTER TICKS/S` line shows the target and the measured speed. Releasing the trigger or bumper lets the flywheel coast to a stop.

Defaults live in `TeamCode/.../config/TunableValues.java` (`SHOOTER_TARGET_TPS`, `SHOOTER_TPS_STEP`, `INTAKE_POWER`, `HOPPER_MANUAL_POWER`).
