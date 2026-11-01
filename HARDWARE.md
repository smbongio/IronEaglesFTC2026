# Hardware map

Source of truth: `IronEaglesConfig.xml`, `IronEaglesHardware.java`, `TunableValues.java`, and Robot Controller logs (latest pull that printed the map: `logs/20261031-123210`).

Config and logs agree on every mapped device. Motor SKUs for drive / intake / shooter are `UnspecifiedMotor` in the config, so they are not listed as a specific REV part.

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
| `intake` | 0 | Intake |
| `hopper` | 1 | Hopper. Code comments: REV Core Hex. |
| `shooter` | 2 | Shooter leader |
| `shooterFollow` | 3 | Shooter follow (same command as `shooter`) |

Motor directions (`*_DIR`, +1 or −1) live in `TunableValues.java`.

## Sensors and other Control Hub devices

| Config name | Plug | Hardware |
|---|---|---|
| `imu` | Built-in I2C **bus 0** (no external jack) | `ControlHubImuBHI260AP`. Mount in code: hub on the **right** side, logo **RIGHT**, USB **DOWN**. |
| `octoquad` | I2C **bus 1**, port 0, addr `0x30` | Digital Chicken OctoQuad FTC MK2, firmware **3.1.0**. Board is upright under the rear of the robot. |
| `blinkin` | Servo / PWM **port 0** | REV Blinkin LED Driver. The LED strip plugs into the Blinkin, not a hub port. |
| `camera` | Control Hub USB | Webcam serial **`4B75BEAF`**. Logs have shown USB permission declined / not streaming. |

## OctoQuad encoder jacks

These are OctoQuad ports, not Control Hub motor ports.

| OctoQuad port | Software name | Encoder | Mount |
|---|---|---|---|
| 0 (`X0`) | Forward / X pod (`OCTOQUAD_FORWARD_PORT`) | REV Through Bore, 8192 ticks/rev, 4 in wheel | 7.75 in behind chassis center |
| 1 (`Y1`) | Strafe / Y pod (`OCTOQUAD_STRAFE_PORT`) | Same | 4.25 in right of chassis center |
| 2–7 | Unused | — | Logs stay at 0 |

Theoretical scale is `25.665` ticks/mm (`OCTOQUAD_*_TICKS_PER_MM`). Real TCP is the X-pod axle ∩ Y-pod axle. Code offsets that point to chassis center.

Heading and field X/Y currently use the **Control Hub IMU + dead-wheel ticks** (`USE_OCTOQUAD_IMU = false`). The OctoQuad IMU / localizer block is still read for diagnostics.

## Not in the robot config

These exist on the robot or at the field, but there is no hub port to list:

- Driver Hub and two gamepads (USB to the Driver Hub)
- Blinkin LED strip (12 V from the Blinkin)
- Exact drive / intake / shooter motor part numbers
- Which Control Hub USB jack the webcam uses

## Runtime check

`logs/20261031-123210` hardware-map dump: **8 motors, 0 servos, 1 webcam, 1 Blinkin**, OctoQuad on bus 1. Nothing else is configured.
