# Hardware map

Source of truth: `IronEaglesConfig.xml`, `IronEaglesHardware.java`, `TunableValues.java`, and Robot Controller logs.

Motor SKUs for drive / intake / shooter are `UnspecifiedMotor` in the config, so they are not listed as a specific REV part.

## Hubs

| Device | Where | Notes |
|---|---|---|
| REV Control Hub v1.0 | Robot (embedded USB portal) | Module **173**, serial `bf9039506746b36b` |
| REV Expansion Hub 2 | RS-485 from Control Hub | Module **2**. Logs have shown it power-cycle |

Config names are the hub port (`chPort0` = Control Hub motor port 0, `ehServo0` = Expansion Hub servo port 0, and so on). What is plugged in lives in the comments in `IronEaglesConfig.xml` and in the tables below. Java fields in `IronEaglesHardware.java` keep the use as their name (`intake`, `shooter`, ...).

## Control Hub motors (ports 0–3)

| Config name | Motor jack | Encoder jack |
|---|---|---|
| `chPort0` | Front right drive (mecanum). Software reverses this motor. | — |
| `chPort1` | Back right drive (mecanum) | — |
| `chPort2` | Back left drive (mecanum) | — |
| `chPort3` | Front left drive (mecanum) | — |

## Expansion Hub motors (ports 0–3)

| Config name | Motor jack | Encoder jack |
|---|---|---|
| `ehPort0` | Intake. Open-loop power (`INTAKE_POWER`, default 50%). | **RIGHT** odometry pod |
| `ehPort1` | No motor | **LEFT** odometry pod |
| `ehPort2` | Shooter. Open-loop power (`SHOOTER_POWER`, default 35%, d-pad ±5%). | Unused |
| `ehPort3` | No motor | **REAR** odometry pod (the X wheel, side-to-side) |

Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`.

Heading and field X/Y come from **three dead wheels only** (FTCLib `HolonomicOdometry`). No IMU, no OctoQuad.

## Servos and other devices

| Config name | Plug | Hardware |
|---|---|---|
| `chServo0` | Control Hub servo / PWM port 0 | REV Blinkin LED Driver. The LED strip plugs into the Blinkin, not a hub port. |
| `ehServo0` | Expansion Hub servo / PWM port 0 | Hopper: REV Smart Servo, **continuous mode** (program with the REV SRS Programmer). Open-loop speed, no position feedback. |
| `camera` | Control Hub USB | Webcam serial **`4B75BEAF`**. Logs have shown USB permission declined / not streaming. |

## Dead-wheel encoders

Hub motor-port encoder jacks. A pod shares its port's config name with whatever motor is on that port.

| Pod | Config name | Java name | Wheel |
|---|---|---|---|
| **LEFT** | `ehPort1` | `leftPod` | Front-to-back, left side |
| **RIGHT** | `ehPort0` | `rightPod` (same device as `intake`) | Front-to-back, right side |
| **REAR** (X wheel) | `ehPort3` | `rearPod` | Side-to-side, rear |

Heading = (RIGHT − LEFT) / `ODO_TRACK_WIDTH_MM`, counter-clockwise positive. Pod wheels are **3.5 in** diameter with 8192 ticks/rev, so scale is **29.33** ticks/mm (`ODO_TICKS_PER_MM`). Measured: LEFT and RIGHT are **8.25 in** apart; `ODO_TRACK_WIDTH_MM` is spin-tuned to 8.205 in. REAR is **9.0 in** behind the midpoint between them (`ODO_REAR_POD_X_MM` = −9.0 in, re-measured on the floor 11/14). Pod misalignment is corrected in software from a 10 ft sideways drag (`ODO_LEFT_PER_SIDEWAYS`, `ODO_RIGHT_PER_SIDEWAYS`, `ODO_REAR_PER_FORWARD`).

In code and on the Driver Hub, X and Y mean **field** coordinates (+X audience, +Y blue); the team's "X wheel" is the REAR pod. Driver Hub shows `FIELD X IN`, `FIELD Y IN`, and `LEFT` / `RIGHT` / `REAR POD TICKS`. The log line is `[ODO] PODS LEFT= RIGHT= REAR= FIELD X= Y=`.

## Not in the robot config

These exist on the robot or at the field, but there is no hub port to list:

- Driver Hub and two gamepads (USB to the Driver Hub)
- Blinkin LED strip (12 V from the Blinkin)
- Exact drive / intake / shooter motor part numbers
- Which Control Hub USB jack the webcam uses
- OctoQuad (removed)
- IMU (not used; the Control Hub's built-in IMU is no longer in the config)

## Runtime check

Hardware-map dump should show **8 motors (`chPort0`–`chPort3`, `ehPort0`–`ehPort3`), 0 servos, 1 CRServo (`ehServo0`), 1 webcam, 1 Blinkin (`chServo0`)**.
