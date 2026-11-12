package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;

import java.util.ArrayList;
import java.util.List;

/**
 * OctoQuad MK2 named "octoquad" for dead-wheel ticks. When
 * {@link TunableValues#USE_OCTOQUAD_IMU} is false, heading is the Control Hub
 * IMU and X/Y are integrated from the pods. The OctoQuad IMU/localizer is
 * still read into the LOC diagnostic block only.
 */
public class OctoQuadOdometry implements ImuInterface {
    private static final String CONFIG_NAME = "octoquad";
    private static final long CALIBRATE_TIMEOUT_MS = 5000L;
    private static final long CALIBRATE_POLL_MS = 50L;
    private static final double LOG_PERIOD_SEC = 0.10;
    private static final int MAX_TICK_STEP = 8000;

    private final OctoQuad octoquad;
    private final IMU hubImu;
    private final OctoQuad.LocalizerDataBlock localizer = new OctoQuad.LocalizerDataBlock();
    private final OctoQuad.EncoderDataBlock encoders = new OctoQuad.EncoderDataBlock();
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime poseTimer = new ElapsedTime();
    private final ElapsedTime uptimeTimer = new ElapsedTime();

    private double headingDegrees;
    private int forwardTicks;
    private int strafeTicks;
    private int posXMm;
    private int posYMm;
    private double velXMmS;
    private double velYMmS;
    private double velHeadingRadS;
    private boolean localizerValid;
    private boolean localizerCrcOk;
    private boolean encoderCrcOk;
    private boolean readFailed;
    private OctoQuad.LocalizerStatus localizerStatus;
    private OctoQuad.LocalizerYawAxis headingAxis;
    private final int[] encoderPositions = new int[OctoQuad.NUM_ENCODERS];
    private final int[] encoderVelocities = new int[OctoQuad.NUM_ENCODERS];
    private int consecutiveBadPackets;
    private String disconnectError;
    private String hubImuError;
    private double lastLoggedHeadingDegrees;
    private int lastLoggedPosXMm;
    private int lastLoggedPosYMm;
    private final int[] lastLoggedEncoderPositions = new int[OctoQuad.NUM_ENCODERS];
    private boolean wasActive;
    private boolean wasRunning;
    private boolean haveLastGoodPose;
    private int lastGoodPosXMm;
    private int lastGoodPosYMm;
    private float lastGoodHeadingRad;
    private double poseXMm;
    private double poseYMm;
    private int lastForwardTicks;
    private int lastStrafeTicks;
    private double lastHeadingRad;
    private boolean haveEncoderBaseline;
    private boolean hubPoseValid;
    private double hubYawDeg;
    private double hubPitchDeg;
    private double hubRollDeg;
    private double hubWxDegS;
    private double hubWyDegS;
    private double hubWzDegS;
    private boolean hubGyroValid;
    private int octoImuDiag;
    private int octoUptimeMs;
    private int lastGoodUptimeMs;
    private int octoRebootCount;
    private boolean haveUptime;
    private boolean uptimeReset;
    private double lastLoggedHubYawDeg;

    public OctoQuadOdometry(HardwareMap hardwareMap) {
        hubImu = tryInitHubImu(hardwareMap);
        octoquad = hardwareMap.tryGet(OctoQuad.class, CONFIG_NAME);
        if (octoquad == null) {
            RobotLog.w("[OCTOQUAD] '%s' not found in the active robot config.", CONFIG_NAME);
            sampleHubImu();
            applyHubHeading();
            RobotLog.i("[IMU] HUB GYRO %s", getHubGyroLine());
            RobotLog.i("[IMU] COMPARE %s", getImuCompareLine());
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
        double[] tcp = rotateVec(TunableValues.OCTOQUAD_TCP_OFFSET_X_MM,
                TunableValues.OCTOQUAD_TCP_OFFSET_Y_MM,
                -TunableValues.OCTOQUAD_POSE_ROTATION_DEG);
        octoquad.setLocalizerTcpOffsetMM_X((float) tcp[0]);
        octoquad.setLocalizerTcpOffsetMM_Y((float) tcp[1]);
        octoquad.setLocalizerImuHeadingScalar(TunableValues.OCTOQUAD_IMU_SCALAR);
        octoquad.setLocalizerVelocityIntervalMS(25);
        octoquad.setI2cRecoveryMode(OctoQuad.I2cRecoveryMode.MODE_1_PERIPH_RST_ON_FRAME_ERR);

        octoquad.resetLocalizerAndCalibrateIMU();
        waitUntilRunning();
        headingAxis = octoquad.getLocalizerHeadingAxisChoice();
        octoquad.resetAllPositions();
        octoquad.setLocalizerPose(startPoseXOctoMm(), startPoseYOctoMm(), startHeadingRad());
        resetAppliedPoseToStart();
        refreshOctoImuDiag();
        sampleHubImu();
        update();
        RobotLog.i("[OCTOQUAD] initialized. i2cBus=%d connection=%s fw=%s axis=%s status=%s  headingSource=%s  startPose x=%.1f y=%.1f in heading=%.1f  poseRot=%.0f  uptime=%s",
                TunableValues.OCTOQUAD_I2C_BUS,
                octoquad.getConnectionInfo(),
                octoquad.getFirmwareVersionString(),
                headingAxis,
                octoquad.getLocalizerStatus(),
                getHeadingSource(),
                TunableValues.START_POSE_X_IN,
                TunableValues.START_POSE_Y_IN,
                headingDegrees,
                TunableValues.OCTOQUAD_POSE_ROTATION_DEG,
                getUptimeStatus());
        RobotLog.i("[OCTOQUAD] POS %s", getEncoderBlockStatus());
        RobotLog.i("[OCTOQUAD] LOC %s", getLocalizerBlockStatus());
        RobotLog.i("[OCTOQUAD] GYRO %s", getOctoGyroLine());
        RobotLog.i("[IMU] HUB GYRO %s", getHubGyroLine());
        RobotLog.i("[IMU] COMPARE %s", getImuCompareLine());
    }

    public void update() {
        localizerValid = false;
        readFailed = false;
        sampleHubImu();
        if (octoquad == null) {
            applyHubHeading();
            return;
        }
        try {
            octoquad.readLocalizerDataAndAllEncoderData(localizer, encoders);
        } catch (RuntimeException e) {
            readFailed = true;
            consecutiveBadPackets++;
            if (consecutiveBadPackets >= 10) {
                disconnectError = "octoquad disconnected";
            }
            applyHubHeadingIfSelected();
            lastHeadingRad = Math.toRadians(headingDegrees);
            return;
        }
        localizerCrcOk = localizer.crcOk;
        localizerStatus = localizer.localizerStatus;
        boolean running = localizerCrcOk && localizerStatus == OctoQuad.LocalizerStatus.RUNNING;
        if (running) {
            consecutiveBadPackets = 0;
            disconnectError = null;
            if (!wasRunning && haveLastGoodPose) {
                sampleUptime(true);
                try {
                    headingAxis = octoquad.getLocalizerHeadingAxisChoice();
                } catch (RuntimeException ignored) {
                    // Keep the last cached axis if the recovery read fails.
                }
                octoquad.setLocalizerPose(lastGoodPosXMm, lastGoodPosYMm, lastGoodHeadingRad);
                localizerValid = false;
                if (useOctoQuadHeading()) {
                    applyOctoPose(lastGoodPosXMm, lastGoodPosYMm, lastGoodHeadingRad, 0.0, 0.0, 0.0);
                }
                RobotLog.w("[OCTOQUAD] localizer recovered; restored pose x=%.1f y=%.1f in heading=%.1f axis=%s",
                        poseXMm / 25.4, poseYMm / 25.4, headingDegrees, headingAxis);
            } else {
                localizerValid = localizer.isDataValid();
                lastGoodPosXMm = localizer.posX_mm;
                lastGoodPosYMm = localizer.posY_mm;
                lastGoodHeadingRad = localizer.heading_rad;
                haveLastGoodPose = true;
                if (useOctoQuadHeading()) {
                    applyOctoPose(
                            localizer.posX_mm,
                            localizer.posY_mm,
                            localizer.heading_rad,
                            localizer.velX_mmS,
                            localizer.velY_mmS,
                            localizer.velHeading_radS);
                }
            }
        } else if (localizerCrcOk) {
            consecutiveBadPackets = 0;
            localizerValid = false;
            if (localizerStatus == OctoQuad.LocalizerStatus.FAULT_NO_IMU) {
                disconnectError = "octoquad IMU fault";
            }
            if (wasRunning) {
                RobotLog.w("[OCTOQUAD] localizer left RUNNING; status=%s. Holding last pose.", localizerStatus);
            }
        } else {
            consecutiveBadPackets++;
            if (consecutiveBadPackets >= 10) {
                disconnectError = "octoquad disconnected";
            }
        }
        wasRunning = running;
        encoderCrcOk = encoders.crcOk;
        if (encoderCrcOk) {
            System.arraycopy(encoders.positions, 0, encoderPositions, 0, encoderPositions.length);
            for (int i = 0; i < encoderVelocities.length; i++) {
                encoderVelocities[i] = encoders.velocities[i];
            }
            forwardTicks = encoderPositions[TunableValues.OCTOQUAD_FORWARD_PORT];
            strafeTicks = encoderPositions[TunableValues.OCTOQUAD_STRAFE_PORT];
            sampleUptime(false);
        }
        applyHubHeadingIfSelected();
        if (!useOctoQuadHeading()) {
            integrateHubPose();
        }
    }

    public String getDisconnectError() {
        return disconnectError;
    }

    public String getUptimeStatus() {
        if (octoquad == null) {
            return "MISSING";
        }
        if (!haveUptime) {
            return "NONE";
        }
        if (octoRebootCount > 0) {
            return String.format("%d ms  reboots=%d", octoUptimeMs, octoRebootCount);
        }
        return String.format("%d ms", octoUptimeMs);
    }

    public boolean hasUptimeReset() {
        return uptimeReset;
    }

    public boolean consumeUptimeReset() {
        boolean reset = uptimeReset;
        uptimeReset = false;
        return reset;
    }

    public List<String> getCurrentHardwareErrors() {
        List<String> errors = new ArrayList<>();
        if (hubImuError != null) {
            errors.add(hubImuError);
        }
        if (octoquad == null) {
            errors.add("octoquad missing");
            return errors;
        }
        if (disconnectError != null) {
            errors.add(disconnectError);
        }
        if (readFailed) {
            errors.add("octoquad i2c read failed");
            return errors;
        }
        if (!encoderCrcOk) {
            errors.add("octoquad encoder crc bad");
        }
        if (!localizerCrcOk) {
            errors.add("octoquad localizer crc bad");
        }
        if (localizerStatus != null && localizerStatus != OctoQuad.LocalizerStatus.RUNNING) {
            errors.add("octoquad localizer " + localizerStatus);
        }
        return errors;
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

    public String getEncoderBlockStatus() {
        return formatEncoderValues(encoderPositions)
                + "  vel "
                + formatEncoderValues(encoderVelocities)
                + encoderCrcSuffix();
    }

    public String getEncoderPositionLine() {
        return formatEncoderValues(encoderPositions) + encoderCrcSuffix();
    }

    public String getEncoderVelocityLine() {
        return formatEncoderValues(encoderVelocities) + encoderCrcSuffix();
    }

    public String getLocalizerBlockStatus() {
        if (octoquad == null) {
            return "MISSING";
        }
        return getLocalizerStatusLine()
                + "  "
                + getLocalizerPoseLine()
                + "  "
                + getLocalizerVelocityLine();
    }

    public String getLocalizerStatusLine() {
        if (octoquad == null) {
            return "MISSING";
        }
        String status = localizerStatus == null ? "NONE" : localizerStatus.name();
        return String.format("status=%s crc=%s valid=%s axis=%s%s",
                status,
                localizerCrcOk ? "OK" : "BAD",
                localizer.isDataValid(),
                headingAxis,
                readFailed ? "  read=FAIL" : "");
    }

    public String getLocalizerPoseLine() {
        if (octoquad == null) {
            return "MISSING";
        }
        return String.format("heading=%.1f  x=%d y=%d mm  (%.1f %.1f in)",
                Math.toDegrees(localizer.heading_rad),
                (int) localizer.posX_mm,
                (int) localizer.posY_mm,
                localizer.posX_mm / 25.4,
                localizer.posY_mm / 25.4);
    }

    public String getLocalizerVelocityLine() {
        if (octoquad == null) {
            return "MISSING";
        }
        return String.format("vx=%.0f vy=%.0f mm/s  w=%.2f rad/s (%.1f deg/s)",
                (double) localizer.velX_mmS,
                (double) localizer.velY_mmS,
                (double) localizer.velHeading_radS,
                Math.toDegrees(localizer.velHeading_radS));
    }

    public String getOctoGyroLine() {
        if (octoquad == null) {
            return "MISSING";
        }
        return String.format("axis=%s  heading=%.1f deg  w=%.1f deg/s  diag=%d",
                headingAxis,
                Math.toDegrees(localizer.heading_rad),
                Math.toDegrees(localizer.velHeading_radS),
                octoImuDiag);
    }

    public String getHubGyroLine() {
        if (hubImu == null) {
            return "MISSING";
        }
        if (!hubGyroValid) {
            return hubImuError != null ? hubImuError : "READ FAIL";
        }
        return String.format("yaw=%.1f pitch=%.1f roll=%.1f  wx=%.1f wy=%.1f wz=%.1f deg/s",
                hubYawDeg, hubPitchDeg, hubRollDeg, hubWxDegS, hubWyDegS, hubWzDegS);
    }

    /**
     * OctoQuad heading vs Control Hub yaw. Hub yaw is reset at init, so both
     * should start at {@link TunableValues#START_HEADING_DEG} and track together.
     * A growing dYaw or a jump on one side only is the stutter signature.
     */
    public String getImuCompareLine() {
        boolean octoOk = octoquad != null && localizerCrcOk;
        if (!octoOk && !hubGyroValid) {
            return "octo=MISSING  hub=MISSING";
        }
        if (!octoOk) {
            return String.format("octo=MISSING  hub yaw=%.1f wz=%.1f deg/s",
                    hubFieldHeadingDeg(), hubWzDegS);
        }
        double octoHeading = Math.toDegrees(localizer.heading_rad);
        double octoW = Math.toDegrees(localizer.velHeading_radS);
        if (!hubGyroValid) {
            return String.format("octo h=%.1f w=%.1f deg/s  hub=%s",
                    octoHeading, octoW, hubImuError != null ? hubImuError : "MISSING");
        }
        double hubHeading = hubFieldHeadingDeg();
        return String.format("octo h=%.1f w=%.1f  hub h=%.1f wz=%.1f  dYaw=%.1f deg",
                octoHeading, octoW, hubHeading, hubWzDegS, wrapDegrees(octoHeading - hubHeading));
    }

    private String encoderCrcSuffix() {
        String suffix = "  crc=" + (encoderCrcOk ? "OK" : "BAD");
        if (readFailed) {
            suffix += "  read=FAIL";
        }
        return suffix;
    }

    private static String formatEncoderValues(int[] values) {
        StringBuilder line = new StringBuilder();
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                line.append("  ");
            }
            if (i == TunableValues.OCTOQUAD_FORWARD_PORT) {
                line.append("X");
            } else if (i == TunableValues.OCTOQUAD_STRAFE_PORT) {
                line.append("Y");
            }
            line.append(i).append("=").append(values[i]);
        }
        return line.toString();
    }

    public int getPosXMm() {
        return posXMm;
    }

    public int getPosYMm() {
        return posYMm;
    }

    public String getPoseStatus() {
        return String.format("x=%.1f y=%.1f in", posXMm / 25.4, posYMm / 25.4);
    }

    /**
     * Heading for field-centric drive. Stick-up is field +Y (red-wall view),
     * not robot-forward, so this is IMU heading minus {@link TunableValues#DRIVE_STICK_ZERO_DEG}.
     */
    public double getDriveHeadingDegrees() {
        double heading = headingDegrees - TunableValues.START_HEADING_DEG
                - TunableValues.DRIVE_STICK_ZERO_DEG;
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading <= -180.0) {
            heading += 360.0;
        }
        return heading;
    }

    public boolean hasLocalizerVelocity() {
        if (useOctoQuadHeading()) {
            return localizerValid;
        }
        return hubPoseValid && encoderCrcOk;
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
     * Writes the encoder position block and localizer block to RobotLog when due.
     * ~10 Hz while pods, pose, heading, or CRC/status are changing; quiet when still.
     */
    public String getHeadingSource() {
        if (useOctoQuadHeading()) {
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
        if (octoquad == null) {
            RobotLog.i("[IMU] source=%s heading=%.1f  HUB GYRO %s",
                    getHeadingSource(), headingDegrees, getHubGyroLine());
            logTimer.reset();
            wasActive = active;
            lastLoggedHeadingDegrees = headingDegrees;
            lastLoggedHubYawDeg = hubYawDeg;
            return;
        }
        refreshOctoImuDiag();
        boolean podsMoved = encoderPositionsChanged();
        double rawHeadingDeg = Math.toDegrees(localizer.heading_rad);
        boolean poseMoved = localizer.posX_mm != lastLoggedPosXMm || localizer.posY_mm != lastLoggedPosYMm;
        boolean headingMoved = Math.abs(rawHeadingDeg - lastLoggedHeadingDegrees) > 0.1;
        RobotLog.i("[OCTOQUAD] POS %s", getEncoderBlockStatus());
        RobotLog.i("[OCTOQUAD] LOC %s", getLocalizerBlockStatus());
        RobotLog.i("[OCTOQUAD] GYRO %s", getOctoGyroLine());
        RobotLog.i("[OCTOQUAD] UPTIME %s", getUptimeStatus());
        RobotLog.i("[IMU] HUB GYRO %s", getHubGyroLine());
        RobotLog.i("[IMU] COMPARE %s", getImuCompareLine());
        RobotLog.i("[OCTOQUAD] COMPARE source=%s pods=%s pose=%s heading=%s  applied x=%.1f y=%.1f in h=%.1f  raw x=%.1f y=%.1f in h=%.1f  hub h=%.1f dYaw=%.1f",
                useOctoQuadHeading() ? "octoquad" : "hub+pods",
                podsMoved,
                poseMoved,
                headingMoved,
                posXMm / 25.4,
                posYMm / 25.4,
                headingDegrees,
                localizer.posX_mm / 25.4,
                localizer.posY_mm / 25.4,
                rawHeadingDeg,
                hubFieldHeadingDeg(),
                wrapDegrees(rawHeadingDeg - hubFieldHeadingDeg()));
        logTimer.reset();
        wasActive = active;
        lastLoggedHeadingDegrees = rawHeadingDeg;
        lastLoggedHubYawDeg = hubYawDeg;
        lastLoggedPosXMm = localizer.posX_mm;
        lastLoggedPosYMm = localizer.posY_mm;
        System.arraycopy(encoderPositions, 0, lastLoggedEncoderPositions, 0, encoderPositions.length);
    }

    @Override
    public boolean isPresent() {
        return octoquad != null || hubImu != null;
    }

    @Override
    public void resetYaw() {
        if (hubImu != null) {
            hubImu.resetYaw();
            headingDegrees = TunableValues.START_HEADING_DEG;
            lastHeadingRad = Math.toRadians(TunableValues.START_HEADING_DEG);
            RobotLog.i("[IMU] Heading restored to start %.1f deg (Back button).",
                    TunableValues.START_HEADING_DEG);
        }
        if (octoquad != null) {
            octoquad.setLocalizerHeading(startHeadingRad());
            if (useOctoQuadHeading()) {
                headingDegrees = TunableValues.START_HEADING_DEG;
            }
            RobotLog.i("[OCTOQUAD] Localizer heading restored to start %.1f deg (Back button).",
                    TunableValues.START_HEADING_DEG);
        }
    }

    @Override
    public double getHeadingDegrees() {
        return headingDegrees;
    }

    private boolean isActive() {
        if (octoquad != null) {
            if (readFailed || !encoderCrcOk || !localizerCrcOk) {
                return true;
            }
            if (localizerStatus != null && localizerStatus != OctoQuad.LocalizerStatus.RUNNING) {
                return true;
            }
            if (Math.abs(Math.toDegrees(localizer.heading_rad) - lastLoggedHeadingDegrees) > 0.1) {
                return true;
            }
            if (localizer.posX_mm != lastLoggedPosXMm || localizer.posY_mm != lastLoggedPosYMm) {
                return true;
            }
            if (hubImuMoved()) {
                return true;
            }
            return encoderPositionsChanged();
        }
        return Math.abs(headingDegrees - lastLoggedHeadingDegrees) > 0.1 || hubImuMoved();
    }

    private boolean hubImuMoved() {
        if (!hubGyroValid) {
            return hubImuError != null;
        }
        if (Math.abs(wrapDegrees(hubYawDeg - lastLoggedHubYawDeg)) > 0.1) {
            return true;
        }
        return Math.abs(hubWzDegS) > 5.0;
    }

    private double hubFieldHeadingDeg() {
        if (!hubGyroValid) {
            return 0.0;
        }
        return wrapDegrees(hubYawDeg + TunableValues.START_HEADING_DEG);
    }

    private boolean encoderPositionsChanged() {
        for (int i = 0; i < encoderPositions.length; i++) {
            if (encoderPositions[i] != lastLoggedEncoderPositions[i]) {
                return true;
            }
        }
        return false;
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

    private IMU tryInitHubImu(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu == null) {
            RobotLog.w("[IMU] Control Hub IMU 'imu' not found. Heading falls back to OctoQuad or 0.");
            return null;
        }
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        headingDegrees = TunableValues.START_HEADING_DEG;
        RobotLog.i("[IMU] Control Hub IMU initialized. logo=%s usb=%s. Yaw zeroed at start heading %.1f.",
                TunableValues.IMU_LOGO_DIRECTION,
                TunableValues.IMU_USB_DIRECTION,
                TunableValues.START_HEADING_DEG);
        return imu;
    }

    private boolean useOctoQuadHeading() {
        return octoquad != null && (TunableValues.USE_OCTOQUAD_IMU || hubImu == null);
    }

    private void applyHubHeadingIfSelected() {
        if (!useOctoQuadHeading()) {
            applyHubHeading();
        }
    }

    private void applyHubHeading() {
        if (!hubGyroValid) {
            return;
        }
        headingDegrees = wrapDegrees(hubYawDeg + TunableValues.START_HEADING_DEG);
    }

    private void sampleHubImu() {
        if (hubImu == null) {
            hubGyroValid = false;
            return;
        }
        try {
            YawPitchRollAngles ypr = hubImu.getRobotYawPitchRollAngles();
            AngularVelocity rates = hubImu.getRobotAngularVelocity(AngleUnit.DEGREES);
            hubYawDeg = ypr.getYaw(AngleUnit.DEGREES);
            hubPitchDeg = ypr.getPitch(AngleUnit.DEGREES);
            hubRollDeg = ypr.getRoll(AngleUnit.DEGREES);
            hubWxDegS = rates.xRotationRate;
            hubWyDegS = rates.yRotationRate;
            hubWzDegS = rates.zRotationRate;
            hubGyroValid = true;
            hubImuError = null;
        } catch (RuntimeException e) {
            hubGyroValid = false;
            hubImuError = "control hub imu disconnected";
        }
    }

    private void sampleUptime(boolean force) {
        if (octoquad == null || readFailed) {
            return;
        }
        if (!force && haveUptime && uptimeTimer.seconds() < 0.10) {
            return;
        }
        try {
            int uptime = octoquad.getUptimeMs();
            uptimeTimer.reset();
            if (haveUptime && uptime + 250 < lastGoodUptimeMs) {
                uptimeReset = true;
                octoRebootCount++;
                RobotLog.w("[OCTOQUAD] uptime reset %d -> %d ms (board rebooted / lost power) reboots=%d",
                        lastGoodUptimeMs, uptime, octoRebootCount);
            }
            octoUptimeMs = uptime;
            lastGoodUptimeMs = uptime;
            haveUptime = true;
        } catch (RuntimeException ignored) {
            // Keep the last cached uptime if this extra I2C read fails.
        }
    }

    private void refreshOctoImuDiag() {
        if (octoquad == null) {
            return;
        }
        try {
            octoImuDiag = octoquad.getIMUDiagParam();
        } catch (RuntimeException ignored) {
            // Keep the last cached diagnostic if this extra I2C read fails.
        }
    }

    private void resetAppliedPoseToStart() {
        poseXMm = startPoseXMm();
        poseYMm = startPoseYMm();
        posXMm = (int) Math.round(poseXMm);
        posYMm = (int) Math.round(poseYMm);
        lastHeadingRad = Math.toRadians(TunableValues.START_HEADING_DEG);
        haveEncoderBaseline = false;
        hubPoseValid = false;
        velXMmS = 0.0;
        velYMmS = 0.0;
        velHeadingRadS = 0.0;
        poseTimer.reset();
    }

    private void integrateHubPose() {
        double headingRad = Math.toRadians(headingDegrees);
        if (!encoderCrcOk) {
            lastHeadingRad = headingRad;
            return;
        }
        if (!haveEncoderBaseline) {
            lastForwardTicks = forwardTicks;
            lastStrafeTicks = strafeTicks;
            lastHeadingRad = headingRad;
            haveEncoderBaseline = true;
            hubPoseValid = true;
            poseTimer.reset();
            return;
        }
        int dFwdTicks = forwardTicks - lastForwardTicks;
        int dStrTicks = strafeTicks - lastStrafeTicks;
        lastForwardTicks = forwardTicks;
        lastStrafeTicks = strafeTicks;
        double dHeading = wrapRadians(headingRad - lastHeadingRad);
        lastHeadingRad = headingRad;
        double dt = poseTimer.seconds();
        poseTimer.reset();
        if (Math.abs(dFwdTicks) > MAX_TICK_STEP || Math.abs(dStrTicks) > MAX_TICK_STEP) {
            RobotLog.w("[OCTOQUAD] skipped encoder step fwd=%d strafe=%d ticks (I2C reset?)",
                    dFwdTicks, dStrTicks);
            velXMmS = 0.0;
            velYMmS = 0.0;
            velHeadingRadS = 0.0;
            return;
        }
        double dFwdMm = dFwdTicks / (double) TunableValues.OCTOQUAD_FORWARD_TICKS_PER_MM;
        double dStrMm = dStrTicks / (double) TunableValues.OCTOQUAD_STRAFE_TICKS_PER_MM;
        double tcpX = TunableValues.OCTOQUAD_TCP_OFFSET_X_MM;
        double tcpY = TunableValues.OCTOQUAD_TCP_OFFSET_Y_MM;
        double dxRobot = dFwdMm - tcpY * dHeading;
        double dyRobot = dStrMm + tcpX * dHeading;
        double theta = headingRad - dHeading / 2.0;
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double dxField = dxRobot * cos - dyRobot * sin;
        double dyField = dxRobot * sin + dyRobot * cos;
        poseXMm += dxField;
        poseYMm += dyField;
        posXMm = (int) Math.round(poseXMm);
        posYMm = (int) Math.round(poseYMm);
        if (dt > 0.001) {
            velXMmS = dxField / dt;
            velYMmS = dyField / dt;
            velHeadingRadS = dHeading / dt;
        }
    }

    private static double wrapRadians(double heading) {
        while (heading > Math.PI) {
            heading -= 2.0 * Math.PI;
        }
        while (heading <= -Math.PI) {
            heading += 2.0 * Math.PI;
        }
        return heading;
    }

    private static double wrapDegrees(double heading) {
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading <= -180.0) {
            heading += 360.0;
        }
        return heading;
    }

    private static OctoQuad.EncoderDirection encoderDirection(int dir) {
        return dir < 0 ? OctoQuad.EncoderDirection.REVERSE : OctoQuad.EncoderDirection.FORWARD;
    }

    private void applyOctoPose(int octoXMm, int octoYMm, float headingRad,
                              double octoVelX, double octoVelY, double velHeading) {
        headingDegrees = Math.toDegrees(headingRad);
        double[] xy = octoToField(octoXMm, octoYMm);
        poseXMm = xy[0];
        poseYMm = xy[1];
        posXMm = (int) Math.round(poseXMm);
        posYMm = (int) Math.round(poseYMm);
        double[] vel = octoToField(octoVelX, octoVelY);
        velXMmS = vel[0];
        velYMmS = vel[1];
        velHeadingRadS = velHeading;
    }

    private static int startPoseXMm() {
        return (int) Math.round(TunableValues.START_POSE_X_IN * 25.4);
    }

    private static int startPoseYMm() {
        return (int) Math.round(TunableValues.START_POSE_Y_IN * 25.4);
    }

    private static int startPoseXOctoMm() {
        return (int) Math.round(fieldToOcto(startPoseXMm(), startPoseYMm())[0]);
    }

    private static int startPoseYOctoMm() {
        return (int) Math.round(fieldToOcto(startPoseXMm(), startPoseYMm())[1]);
    }

    private static float startHeadingRad() {
        return (float) Math.toRadians(TunableValues.START_HEADING_DEG);
    }

    /**
     * OctoQuad onboard XY is rotated {@link TunableValues#OCTOQUAD_POSE_ROTATION_DEG}
     * from the FTC field. Heading is already field yaw and is not rotated.
     */
    private static double[] octoToField(double x, double y) {
        double[] xy = rotateVec(x, y, TunableValues.OCTOQUAD_POSE_ROTATION_DEG);
        xy[0] *= TunableValues.OCTOQUAD_POSE_X_DIR;
        xy[1] *= TunableValues.OCTOQUAD_POSE_Y_DIR;
        return xy;
    }

    private static double[] fieldToOcto(double x, double y) {
        return rotateVec(x * TunableValues.OCTOQUAD_POSE_X_DIR,
                y * TunableValues.OCTOQUAD_POSE_Y_DIR,
                -TunableValues.OCTOQUAD_POSE_ROTATION_DEG);
    }

    private static double[] rotateVec(double x, double y, double deg) {
        double rad = Math.toRadians(deg);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        return new double[] { x * cos - y * sin, x * sin + y * cos };
    }
}
