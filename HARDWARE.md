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
| `backRight` | 1 | Mecanum. Encoder jack is the **back (side-to-side)** odometry pod. Drive code never reads this motor's encoder. |
| `backLeft` | 2 | Mecanum |
| `frontLeft` | 3 | Mecanum |

## Expansion Hub motors (ports 0–3)

| Config name | Port | Use |
|---|---|---|
| `intake` | 0 | Intake motor. **No intake encoder** — open-loop power (`INTAKE_POWER`, default 25%). Encoder jack is the **left (front-to-back)** odometry pod. |
| `hopper` | 1 | Hopper motor (REV Core Hex). Uses its own encoder for hopper steps. |
| `rightPod` | 2 | **No motor.** Encoder jack is the **right (front-to-back)** odometry pod. |
| `shooter` | 3 | Shooter. **Uses its own encoder** for velocity (`RUN_USING_ENCODER`). |

Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`.

Heading and field X/Y come from **three dead wheels only** (FTCLib `HolonomicOdometry`). No IMU, no OctoQuad.

## Sensors and other Control Hub devices

| Config name | Plug | Hardware |
|---|---|---|
| `blinkin` | Servo / PWM **port 0** | REV Blinkin LED Driver. The LED strip plugs into the Blinkin, not a hub port. |
| `camera` | Control Hub USB | Webcam serial **`4B75BEAF`**. Logs have shown USB permission declined / not streaming. |

## Dead-wheel encoders

Hub motor-port encoder jacks. One encoder jack per motor port, so a pod on a motor's port shares that port's config name.

| Hub port | Java name | Config port name | Wheel |
|---|---|---|---|
| EH 0 | `leftPod` | `intake` | Front-to-back, left side |
| EH 2 | `rightPod` | `rightPod` | Front-to-back, right side |
| CH 1 | `backPod` | `backRight` | Side-to-side, rear |

Heading = (right − left) / `ODO_TRACK_WIDTH_MM`, counter-clockwise positive. Scale is `25.665` ticks/mm (`ODO_TICKS_PER_MM`). `ODO_BACK_POD_X_MM` is the back pod's distance in front of center (negative = behind). Track width is a placeholder until measured.

## Not in the robot config

These exist on the robot or at the field, but there is no hub port to list:

- Driver Hub and two gamepads (USB to the Driver Hub)
- Blinkin LED strip (12 V from the Blinkin)
- Exact drive / intake / shooter motor part numbers
- Which Control Hub USB jack the webcam uses
- OctoQuad (removed)
- IMU (not used; the Control Hub's built-in IMU is no longer in the config)

## Runtime check

Hardware-map dump should show **8 motors, 0 servos, 1 webcam, 1 Blinkin**. Expansion Hub motors are intake, hopper, rightPod, shooter.
