package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;

/**
 * OctoQuad MK2 named "octoquad". Heading comes from the onboard IMU via the
 * localizer. Missing board falls back to heading 0 (robot-centric).
 */
public class OctoQuadOdometry implements ImuInterface {
    private static final String CONFIG_NAME = "octoquad";
    private static final long CALIBRATE_TIMEOUT_MS = 5000L;
    private static final long CALIBRATE_POLL_MS = 50L;
    private static final double LOG_PERIOD_SEC = 0.10;

    private final OctoQuad octoquad;
    private final IMU hubImu;
    private final OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private final ElapsedTime logTimer = new ElapsedTime();

    private double headingDegrees;
    private int forwardTicks;
    private int strafeTicks;
    private int posXMm;
    private int posYMm;
    private double velXMmS;
    private double velYMmS;
    private double velHeadingRadS;
    private boolean localizerValid;
    private int consecutiveBadPackets;
    private String disconnectError;
    private double lastLoggedHeadingDegrees;
    private int lastLoggedForwardTicks;
    private int lastLoggedStrafeTicks;
    private boolean wasActive;

    public OctoQuadOdometry(HardwareMap hardwareMap) {
        octoquad = hardwareMap.tryGet(OctoQuad.class, CONFIG_NAME);
        if (octoquad == null) {
            RobotLog.w("[OCTOQUAD] '%s' not found in the active robot config.", CONFIG_NAME);
            hubImu = tryInitHubImu(hardwareMap);
            return;
        }
        hubImu = null;

        octoquad.setSingleEncoderDirection(
                TunableValues.OCTOQUAD_FORWARD_PORT, encoderDirection(TunableValues.OCTOQUAD_FORWARD_DIR));
        octoquad.setSingleEncoderDirection(
                TunableValues.OCTOQUAD_STRAFE_PORT, encoderDirection(TunableValues.OCTOQUAD_STRAFE_DIR));
        octoquad.setLocalizerPortX(TunableValues.OCTOQUAD_FORWARD_PORT);
        octoquad.setLocalizerPortY(TunableValues.OCTOQUAD_STRAFE_PORT);
        octoquad.setLocalizerCountsPerMM_X(TunableValues.OCTOQUAD_FORWARD_TICKS_PER_MM);
        octoquad.setLocalizerCountsPerMM_Y(TunableValues.OCTOQUAD_STRAFE_TICKS_PER_MM);
        octoquad.setLocalizerTcpOffsetMM_X(TunableValues.OCTOQUAD_TCP_OFFSET_X_MM);
        octoquad.setLocalizerTcpOffsetMM_Y(TunableValues.OCTOQUAD_TCP_OFFSET_Y_MM);
        octoquad.setLocalizerImuHeadingScalar(TunableValues.OCTOQUAD_IMU_SCALAR);
        octoquad.setLocalizerVelocityIntervalMS(25);
        octoquad.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        octoquad.resetLocalizerAndCalibrateIMU();
        waitUntilRunning();
        octoquad.resetAllPositions();
        update();
        RobotLog.i("[OCTOQUAD] initialized. i2cBus=%d connection=%s fw=%s axis=%s status=%s",
                TunableValues.OCTOQUAD_I2C_BUS,
                octoquad.getConnectionInfo(),
                octoquad.getFirmwareVersionString(),
                octoquad.getLocalizerHeadingAxisChoice(),
                octoquad.getLocalizerStatus());
        RobotLog.i("[OCTOQUAD] post-cal snapshot status=%s heading=%.1f  pose x=%d y=%d mm  vx=%.0f vy=%.0f mm/s  w=%.2f rad/s  crc=%s valid=%s",
                octoquad.getLocalizerStatus(),
                headingDegrees,
                posXMm,
                posYMm,
                velXMmS,
                velYMmS,
                velHeadingRadS,
                localizer.crcOk,
                localizerValid);
    }

    public void update() {
        localizerValid = false;
        if (octoquad == null) {
            if (hubImu != null) {
                try {
                    headingDegrees = hubImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    disconnectError = null;
                } catch (RuntimeException e) {
                    disconnectError = "imu disconnected";
                }
            }
            return;
        }
        try {
            octoquad.readLocalizerDataAndAllEncoderData(localizer, encoders);
        } catch (RuntimeException e) {
            consecutiveBadPackets++;
            if (consecutiveBadPackets >= 10) {
                disconnectError = "octoquad disconnected";
            }
            return;
        }
        if (localizer.crcOk) {
            consecutiveBadPackets = 0;
            disconnectError = null;
            headingDegrees = Math.toDegrees(localizer.heading_rad);
            posXMm = localizer.posX_mm;
            posYMm = localizer.posY_mm;
            velXMmS = localizer.velX_mmS;
            velYMmS = localizer.velY_mmS;
            velHeadingRadS = localizer.velHeading_radS;
            localizerValid = localizer.isDataValid();
            if (localizer.localizerStatus == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                disconnectError = "octoquad IMU fault";
            }
        } else {
            consecutiveBadPackets++;
            if (consecutiveBadPackets >= 10) {
                disconnectError = "octoquad disconnected";
            }
        }
        if (encoders.crcOk) {
            forwardTicks = encoders.positions[TunableValues.OCTOQUAD_FORWARD_PORT];
            strafeTicks = encoders.positions[TunableValues.OCTOQUAD_STRAFE_PORT];
        }
    }

    public String getDisconnectError() {
        return disconnectError;
    }

    public int getForwardTicks() {
        return forwardTicks;
    }

    public int getStrafeTicks() {
        return strafeTicks;
    }

    public String getPodTickStatus() {
        return String.format("fwd %d  strafe %d", forwardTicks, strafeTicks);
    }

    public int getPosXMm() {
        return posXMm;
    }

    public int getPosYMm() {
        return posYMm;
    }

    public String getPoseStatus() {
        return String.format("x=%d y=%d mm", posXMm, posYMm);
    }

    public boolean hasLocalizerVelocity() {
        return localizerValid;
    }

    public double getVelXMmS() {
        return velXMmS;
    }

    public double getVelYMmS() {
        return velYMmS;
    }

    public double getVelHeadingRadS() {
        return velHeadingRadS;
    }

    public String getVelocityStatus() {
        return String.format("vx=%.0f vy=%.0f mm/s  w=%.2f rad/s", velXMmS, velYMmS, velHeadingRadS);
    }

    /**
     * Writes heading, raw pod ticks, and localizer pose to RobotLog when due.
     * Same cadence as mecanum wheel diagnostics: ~10 Hz while moving, quiet when still.
     */
    public String getHeadingSource() {
        if (octoquad != null) {
            return "OctoQuad";
        }
        if (hubImu != null) {
            return "Control Hub IMU";
        }
        return "MISSING";
    }

    public void logOdometryDiagnostics() {
        if (octoquad == null && hubImu == null) {
            return;
        }
        boolean active = isActive();
        if (!active && !wasActive) {
            return;
        }
        if (active == wasActive && logTimer.seconds() < LOG_PERIOD_SEC) {
            return;
        }
        RobotLog.i("[OCTOQUAD] source=%s heading=%.1f  %s  pose x=%d y=%d mm  %s",
                getHeadingSource(), headingDegrees, getPodTickStatus(), posXMm, posYMm, getVelocityStatus());
        logTimer.reset();
        wasActive = active;
        lastLoggedHeadingDegrees = headingDegrees;
        lastLoggedForwardTicks = forwardTicks;
        lastLoggedStrafeTicks = strafeTicks;
    }

    @Override
    public boolean isPresent() {
        return octoquad != null || hubImu != null;
    }

    @Override
    public void resetYaw() {
        if (octoquad != null) {
            octoquad.setLocalizerHeading(0.0f);
            headingDegrees = 0.0;
            RobotLog.i("[OCTOQUAD] Heading zeroed (Back button).");
            return;
        }
        if (hubImu != null) {
            hubImu.resetYaw();
            headingDegrees = 0.0;
            RobotLog.i("[IMU] Heading zeroed (Back button).");
        }
    }

    @Override
    public double getHeadingDegrees() {
        return headingDegrees;
    }

    private boolean isActive() {
        return Math.abs(headingDegrees - lastLoggedHeadingDegrees) > 0.1
                || forwardTicks != lastLoggedForwardTicks
                || strafeTicks != lastLoggedStrafeTicks;
    }

    private void waitUntilRunning() {
        long startMs = System.currentTimeMillis();
        while (System.currentTimeMillis() - startMs < CALIBRATE_TIMEOUT_MS) {
            OctoQuad.LocalizerStatus status = octoquad.getLocalizerStatus();
            if (status == OctoQuad.LocalizerStatus.RUNNING) {
                return;
            }
            if (status == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                RobotLog.w("[OCTOQUAD] Localizer FAULT_NO_IMU. Heading stays 0.");
                return;
            }
            try {
                Thread.sleep(CALIBRATE_POLL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
        RobotLog.w("[OCTOQUAD] IMU calibration timed out. status=%s", octoquad.getLocalizerStatus());
    }

    private static IMU tryInitHubImu(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu == null) {
            RobotLog.w("[OCTOQUAD] Control Hub IMU also missing. Driving will be robot-centric.");
            return null;
        }
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        RobotLog.w("[OCTOQUAD] Using Control Hub IMU for heading until 'octoquad' is in the active config.");
        return imu;
    }

    private static OctoQuad.EncoderDirection encoderDirection(int dir) {
        return dir < 0 ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD;
    }
}
