# Hardware map

Source of truth: `IronEaglesConfig.xml`, `IronEaglesHardware.java`, `TunableValues.java`, and Robot Controller logs.

Motor SKUs for drive / intake / shooter are `UnspecifiedMotor` in the config, so they are not listed as a specific REV part.

## Hubs

| Device | Where | Notes |
|---|---|---|
| REV Control Hub v1.0 | Robot (embedded USB portal) | Module **173**, serial `bf9039506746b36b` |
| REV Expansion Hub 2 | RS-485 from Control Hub | Module **2**. Logs have shown it power-cycle |

## Control Hub motors (ports 0–3)

| Config name | Port | Use |
|---|---|---|
| `frontRight` | 0 | Mecanum. Software reverses this motor (mounted opposite the others). |
| `backRight` | 1 | Mecanum |
| `backLeft` | 2 | Mecanum |
| `frontLeft` | 3 | Mecanum |

## Expansion Hub motors (ports 0–3)

| Config name | Port | Use |
|---|---|---|
| `intake` | 0 | Intake motor. Open-loop power (`INTAKE_POWER`, default 25%). Encoder jack is the **RIGHT** odometry pod. |
| `leftPod` | 1 | **No motor.** Encoder jack is the **LEFT** odometry pod. |
| — | 2 | **Free.** Motor jack and encoder jack both unused. |
| `shooter` | 3 | Shooter motor. **Open-loop power** (`SHOOTER_POWER`, default 35%, d-pad ±5%). Encoder jack is the **REAR** odometry pod, so the shooter has no speed feedback. |

Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`.

Heading and field X/Y come from **three dead wheels only** (FTCLib `HolonomicOdometry`). No IMU, no OctoQuad.

## Sensors and other Control Hub devices

| Config name | Plug | Hardware |
|---|---|---|
| `blinkin` | Servo / PWM **port 0** | REV Blinkin LED Driver. The LED strip plugs into the Blinkin, not a hub port. |
| `hopper` | Servo / PWM **port 2** | REV Smart Servo, **continuous mode** (program with the REV SRS Programmer). Open-loop speed, no position feedback. |
| `camera` | Control Hub USB | Webcam serial **`4B75BEAF`**. Logs have shown USB permission declined / not streaming. |

## Dead-wheel encoders

Hub motor-port encoder jacks. One encoder jack per motor port, so a pod on a motor's port shares that port's config name.

| Pod | Hub port | Java name | Config port name | Wheel |
|---|---|---|---|---|
| **LEFT** | EH 1 | `leftPod` | `leftPod` | Front-to-back, left side |
| **RIGHT** | EH 0 | `rightPod` | `intake` | Front-to-back, right side |
| **REAR** | EH 3 | `rearPod` | `shooter` | Side-to-side, rear |

Heading = (RIGHT − LEFT) / `ODO_TRACK_WIDTH_MM`, counter-clockwise positive. Pod wheels are **3.5 in** diameter with 8192 ticks/rev, so scale is **29.33** ticks/mm (`ODO_TICKS_PER_MM`). Measured: LEFT and RIGHT are **8.25 in** apart (`ODO_TRACK_WIDTH_MM`). REAR is **9.0 in** behind the midpoint between them (`ODO_REAR_POD_X_MM` = −9.0 in, re-measured on the floor 11/14).

X and Y always mean **field** coordinates (+X audience, +Y blue), never a wheel. Driver Hub shows `FIELD X IN`, `FIELD Y IN`, and `LEFT` / `RIGHT` / `REAR POD TICKS`. The log line is `[ODO] PODS LEFT= RIGHT= REAR= FIELD X= Y=`.

## Not in the robot config

These exist on the robot or at the field, but there is no hub port to list:

- Driver Hub and two gamepads (USB to the Driver Hub)
- Blinkin LED strip (12 V from the Blinkin)
- Exact drive / intake / shooter motor part numbers
- Which Control Hub USB jack the webcam uses
- OctoQuad (removed)
- IMU (not used; the Control Hub's built-in IMU is no longer in the config)

## Runtime check

Hardware-map dump should show **7 motors, 0 servos, 1 CRServo (hopper), 1 webcam, 1 Blinkin**. Expansion Hub motors are intake, leftPod, shooter.
