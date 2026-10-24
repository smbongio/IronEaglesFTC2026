package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
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
    private final OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private final ElapsedTime logTimer = new ElapsedTime();

    private double headingDegrees;
    private int forwardTicks;
    private int strafeTicks;
    private int posXMm;
    private int posYMm;
    private double lastLoggedHeadingDegrees;
    private int lastLoggedForwardTicks;
    private int lastLoggedStrafeTicks;
    private boolean wasActive;

    public OctoQuadOdometry(HardwareMap hardwareMap) {
        octoquad = hardwareMap.tryGet(OctoQuad.class, CONFIG_NAME);
        if (octoquad == null) {
            RobotLog.w("[OCTOQUAD] '%s' not found. Driving will be robot-centric.", CONFIG_NAME);
            return;
        }

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
    }

    public void update() {
        if (octoquad == null) {
            return;
        }
        octoquad.readLocalizerDataAndAllEncoderData(localizer, encoders);
        if (localizer.crcOk) {
            headingDegrees = Math.toDegrees(localizer.heading_rad);
            posXMm = localizer.posX_mm;
            posYMm = localizer.posY_mm;
        }
        if (encoders.crcOk) {
            forwardTicks = encoders.positions[TunableValues.OCTOQUAD_FORWARD_PORT];
            strafeTicks = encoders.positions[TunableValues.OCTOQUAD_STRAFE_PORT];
        }
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

    /**
     * Writes heading, raw pod ticks, and localizer pose to RobotLog when due.
     * Same cadence as mecanum wheel diagnostics: ~10 Hz while moving, quiet when still.
     */
    public void logOdometryDiagnostics() {
        if (octoquad == null) {
            return;
        }
        boolean active = isActive();
        if (!active && !wasActive) {
            return;
        }
        if (active == wasActive && logTimer.seconds() < LOG_PERIOD_SEC) {
            return;
        }
        RobotLog.i("[OCTOQUAD] heading=%.1f  %s  pose x=%d y=%d mm",
                headingDegrees, getPodTickStatus(), posXMm, posYMm);
        logTimer.reset();
        wasActive = active;
        lastLoggedHeadingDegrees = headingDegrees;
        lastLoggedForwardTicks = forwardTicks;
        lastLoggedStrafeTicks = strafeTicks;
    }

    @Override
    public boolean isPresent() {
        return octoquad != null;
    }

    @Override
    public void resetYaw() {
        if (octoquad == null) {
            return;
        }
        octoquad.setLocalizerHeading(0.0f);
        headingDegrees = 0.0;
        RobotLog.i("[OCTOQUAD] Heading zeroed (Back button).");
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

    private static OctoQuad.EncoderDirection encoderDirection(int dir) {
        return dir < 0 ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD;
    }
}
