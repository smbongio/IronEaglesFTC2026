package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.CameraPoseCorrector;
import org.firstinspires.ftc.teamcode.subsystems.CellLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.DeadWheelOdometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The four Autonomous programs. Plan, in red field numbers (blue is rotated 180°):
 * <ol>
 *   <li>Back 12 in toward our cell (shooter and camera are on the back).</li>
 *   <li>Cell down only: wait for the partner to tip the hive.</li>
 *   <li>Spin the shooter up, then turn the hopper 180° twice.</li>
 *   <li>Drive along the park lane to the loading zone edge nearest our start wall,
 *       front toward the alliance wall, and push into the wall to square up.</li>
 *   <li>Camera check: if our cell's tags are seen, correct X and re-park until in
 *       tolerance; if nothing is seen, stop.</li>
 * </ol>
 * Throughout, cell tag sightings correct X/Y every few seconds ({@link CameraPoseCorrector}).
 * Records the alliance and the pose every loop so TeleOp starts where Autonomous ended.
 */
public abstract class IronEaglesAutoBase extends LinearOpMode {

    private static final double PARK_HEADING_RED_DEG = -90.0;

    protected abstract Alliance alliance();

    /** True: start on the audience wall facing the up cell. False: far wall, wait for the tip. */
    protected abstract boolean cellUp();

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime hopperTimer = new ElapsedTime();
    private double hopperStopMs = -1.0;
    private IronEaglesHardware robot;
    private DeadWheelOdometry odometry;
    private AprilTagVision vision;
    private CellLocalizer cells;
    private final CameraPoseCorrector cameraFix = new CameraPoseCorrector();
    private Alliance side;
    private String program;
    private String step = "INIT";

    @Override
    public void runOpMode() {
        side = alliance();
        boolean up = cellUp();
        program = side + (up ? " CELL UP" : " CELL DOWN");
        RobotLog.i("OpMode Initializing: Auto %s", program);

        double redStartX = up ? TunableValues.START_POSE_X_IN : TunableValues.START_FAR_POSE_X_IN;
        double redStartY = TunableValues.START_POSE_Y_IN;
        double redStartH = up ? TunableValues.START_HEADING_DEG : TunableValues.START_FAR_HEADING_DEG;

        robot = new IronEaglesHardware();
        robot.init(hardwareMap);
        odometry = new DeadWheelOdometry(robot.leftPod, robot.rightPod, robot.rearPod,
                side.x(redStartX), side.y(redStartY), side.heading(redStartH));
        vision = new AprilTagVision(hardwareMap, robot.camera);
        cells = new CellLocalizer();
        MatchState.alliance = side;
        MatchState.clearAutoPose();
        robot.getLed().setAlliance(side);

        String startWall = (up == (side == Alliance.RED)) ? "AUDIENCE" : "FAR";
        while (opModeInInit()) {
            odometry.update();
            List<AprilTagDetection> detections = vision.sample();
            telemetry.addData("PROGRAM", program);
            telemetry.addData("PLACE ROBOT", "intake bumper on %s wall, side bumper 52 in from %s wall",
                    startWall, side);
            telemetry.addData("START POSE", "X %.1f Y %.1f heading %.1f",
                    side.x(redStartX), side.y(redStartY), side.heading(redStartH));
            telemetry.addData("ODOMETRY", odometry.isPresent() ? "OK" : "MISSING - will shoot without moving");
            telemetry.addData("CAMERA", vision.getCameraStateName());
            addCellSeenTelemetry(detections);
            if (!robot.missingDevices.isEmpty()) {
                telemetry.addData("MISSING", String.join(", ", robot.missingDevices));
            }
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            vision.close();
            return;
        }

        matchTimer.reset();
        RobotLog.i("[AUTO] %s started at X=%.1f Y=%.1f heading=%.1f", program,
                side.x(redStartX), side.y(redStartY), side.heading(redStartH));
        try {
            hopperToVertical();
            double redShootX = redStartX + (up ? -1.0 : 1.0) * TunableValues.AUTO_MOVE_TO_CELL_IN;
            setStep("MOVE TO CELL");
            moveTo(redShootX, redStartY, redStartH, TunableValues.AUTO_POS_TOL_IN);

            if (!up) {
                setStep("WAIT FOR TIP");
                waitSeconds(TunableValues.AUTO_CELL_DOWN_WAIT_S);
            }
            shoot();

            double redParkX = up ? TunableValues.LOADING_ZONE_AUDIENCE_EDGE_X_IN
                    : TunableValues.LOADING_ZONE_FAR_EDGE_X_IN;
            park(redShootX, redParkX);
            checkParkWithCamera(redParkX);

            while (opModeIsActive()) {
                updateRobot();
                writeTelemetry(null);
            }
        } finally {
            robot.getDrivetrain().stop();
            robot.getShooter().stop();
            robot.getShooter().update();
            robot.getHopper().stop();
            vision.close();
            if (MatchState.hasAutoPose) {
                RobotLog.i("[AUTO] %s ended (%s) at FIELD X=%.1f Y=%.1f heading=%.1f", program, step,
                        MatchState.autoXIn, MatchState.autoYIn, MatchState.autoHeadingDeg);
            }
        }
    }

    /** Starts the 90° turn from the loaded (horizontal) to the rest (vertical) position; updateRobot() stops it. */
    private void hopperToVertical() {
        RobotLog.i("[AUTO] Hopper quarter turn to vertical (%d ms)", TunableValues.AUTO_HOPPER_QUARTER_TURN_MS);
        robot.getHopper().runHopper(TunableValues.AUTO_HOPPER_POWER * TunableValues.AUTO_HOPPER_QUARTER_TURN_DIR);
        hopperTimer.reset();
        hopperStopMs = TunableValues.AUTO_HOPPER_QUARTER_TURN_MS;
    }

    private void shoot() {
        if (hopperStopMs >= 0) {
            setStep("HOPPER TO VERTICAL");
            while (opModeIsActive() && hopperStopMs >= 0) {
                updateRobot();
                writeTelemetry(null);
            }
        }
        setStep("SPIN UP");
        robot.getShooter().runShooter(1.0);
        waitSeconds(TunableValues.AUTO_SPINUP_S);
        for (int turn = 1; turn <= TunableValues.AUTO_HOPPER_TURNS && opModeIsActive(); turn++) {
            setStep("HOPPER TURN " + turn);
            robot.getHopper().runHopper(TunableValues.AUTO_HOPPER_POWER);
            waitSeconds(TunableValues.AUTO_HOPPER_HALF_TURN_MS / 1000.0);
            robot.getHopper().stop();
            if (turn < TunableValues.AUTO_HOPPER_TURNS) {
                waitSeconds(TunableValues.AUTO_HOPPER_PAUSE_MS / 1000.0);
            }
        }
        robot.getShooter().stop();
    }

    private void park(double redShootX, double redParkX) {
        double lane = TunableValues.AUTO_PARK_LANE_Y_IN;
        setStep("TO PARK LANE");
        moveTo(redShootX, lane, PARK_HEADING_RED_DEG, TunableValues.AUTO_WAYPOINT_TOL_IN);
        setStep("ALONG PARK LANE");
        moveTo(redParkX, lane, PARK_HEADING_RED_DEG, TunableValues.AUTO_WAYPOINT_TOL_IN);
        approachAndSquare(redParkX);
    }

    private void approachAndSquare(double redParkX) {
        setStep("TO LOADING ZONE");
        moveTo(redParkX, redWallY() + TunableValues.AUTO_WALL_APPROACH_IN, PARK_HEADING_RED_DEG,
                TunableValues.AUTO_POS_TOL_IN);
        squareOnWall();
    }

    /** Pushes the intake bumper into the alliance wall, then trusts the wall for Y and heading. */
    private void squareOnWall() {
        if (!odometry.isPresent() || timeLeft() < TunableValues.AUTO_END_MARGIN_S) {
            return;
        }
        setStep("SQUARE ON WALL");
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < TunableValues.AUTO_WALL_SQUARE_MS) {
            robot.getDrivetrain().drive(0.0, TunableValues.AUTO_WALL_SQUARE_POWER, 0.0);
            updateRobot();
            writeTelemetry(null);
        }
        robot.getDrivetrain().stop();
        if (!opModeIsActive()) {
            return;
        }
        odometry.update();
        double before = odometry.getPosYMm() / 25.4;
        double beforeHeading = odometry.getHeadingDegrees();
        odometry.setPoseInches(odometry.getPosXMm() / 25.4, side.y(redWallY()),
                side.heading(PARK_HEADING_RED_DEG));
        cameraFix.discardSightings();
        savePose();
        RobotLog.i("[AUTO] Squared on wall: Y %.1f -> %.1f, heading %.1f -> %.1f", before,
                side.y(redWallY()), beforeHeading, side.heading(PARK_HEADING_RED_DEG));
    }

    private void checkParkWithCamera(double redParkX) {
        if (!odometry.isPresent() || !vision.isPresent()) {
            setStep("PARKED (camera not used)");
            return;
        }
        for (int attempt = 1; attempt <= TunableValues.AUTO_VISION_MAX_TRIES && opModeIsActive(); attempt++) {
            if (timeLeft() < TunableValues.AUTO_END_MARGIN_S + TunableValues.AUTO_VISION_WINDOW_MS / 1000.0) {
                setStep("PARKED (out of time for camera)");
                return;
            }
            setStep("CAMERA CHECK " + attempt);
            Double fieldX = visionFieldX();
            if (fieldX == null) {
                setStep("PARKED (camera not used)");
                return;
            }
            double redX = side.x(fieldX);
            odometry.setPositionInches(fieldX, odometry.getPosYMm() / 25.4);
            cameraFix.discardSightings();
            savePose();
            if (Math.abs(redX - redParkX) <= TunableValues.AUTO_PARK_TOL_IN) {
                setStep("PARKED (camera confirmed)");
                RobotLog.i("[AUTO] Camera confirms park: X=%.1f (zone edge %.1f)", fieldX, side.x(redParkX));
                return;
            }
            RobotLog.i("[AUTO] Camera says X=%.1f, zone edge %.1f: re-parking", fieldX, side.x(redParkX));
            approachAndSquare(redParkX);
        }
        setStep("PARKED (camera tries used up)");
    }

    /**
     * Median robot field X from our cell's tags over a short window, or null when too few
     * frames saw a cell or the fix disagrees with odometry by more than the allowed correction.
     */
    private Double visionFieldX() {
        List<Double> xs = new ArrayList<>();
        long lastFrame = -1;
        ElapsedTime window = new ElapsedTime();
        while (opModeIsActive() && window.milliseconds() < TunableValues.AUTO_VISION_WINDOW_MS) {
            updateRobot();
            List<AprilTagDetection> detections = vision.sample();
            if (!detections.isEmpty() && detections.get(0).frameAcquisitionNanoTime != lastFrame) {
                lastFrame = detections.get(0).frameAcquisitionNanoTime;
                for (CellLocalizer.Sighting s : cells.sightings(detections, odometry.getHeadingDegrees())) {
                    xs.add(s.robotXIn());
                }
            }
            writeTelemetry(detections);
            sleep(10);
        }
        if (xs.size() < TunableValues.AUTO_VISION_MIN_FRAMES) {
            RobotLog.i("[AUTO] Camera saw our cell in %d frames (need %d); keeping odometry.",
                    xs.size(), TunableValues.AUTO_VISION_MIN_FRAMES);
            return null;
        }
        Collections.sort(xs);
        double median = xs.get(xs.size() / 2);
        double odoX = odometry.getPosXMm() / 25.4;
        if (Math.abs(median - odoX) > TunableValues.AUTO_VISION_MAX_CORRECTION_IN) {
            RobotLog.w("[AUTO] Camera X=%.1f is %.1f in from odometry X=%.1f; ignoring it.",
                    median, median - odoX, odoX);
            return null;
        }
        RobotLog.i("[AUTO] Camera X=%.1f from %d sightings (odometry X=%.1f)", median, xs.size(), odoX);
        return median;
    }

    /**
     * Drives to a red-frame pose (converted for our alliance) with field-frame P control.
     * Returns false on timeout, missing odometry, or the end of Autonomous.
     */
    private boolean moveTo(double redX, double redY, double redHeading, double tolIn) {
        if (!odometry.isPresent()) {
            RobotLog.w("[AUTO] %s skipped: odometry missing.", step);
            return false;
        }
        double tx = side.x(redX);
        double ty = side.y(redY);
        double th = side.heading(redHeading);
        ElapsedTime timer = new ElapsedTime();
        boolean reached = false;
        while (opModeIsActive() && timer.seconds() < TunableValues.AUTO_MOVE_TIMEOUT_S
                && timeLeft() > TunableValues.AUTO_END_MARGIN_S) {
            updateRobot();
            double ex = tx - odometry.getPosXMm() / 25.4;
            double ey = ty - odometry.getPosYMm() / 25.4;
            double eh = wrapDegrees(th - odometry.getHeadingDegrees());
            double dist = Math.hypot(ex, ey);
            if (dist <= tolIn && Math.abs(eh) <= TunableValues.AUTO_HEADING_TOL_DEG) {
                reached = true;
                break;
            }
            double vx = 0.0;
            double vy = 0.0;
            if (dist > tolIn) {
                double speed = clip(TunableValues.AUTO_DRIVE_P * dist,
                        TunableValues.AUTO_DRIVE_MIN_POWER, TunableValues.AUTO_DRIVE_MAX_POWER);
                vx = ex / dist * speed;
                vy = ey / dist * speed;
            }
            double h = Math.toRadians(odometry.getHeadingDegrees());
            double forward = vx * Math.cos(h) + vy * Math.sin(h);
            double left = -vx * Math.sin(h) + vy * Math.cos(h);
            double turnCcw = clip(TunableValues.AUTO_TURN_P * eh,
                    -TunableValues.AUTO_TURN_MAX_POWER, TunableValues.AUTO_TURN_MAX_POWER);
            robot.getDrivetrain().drive(-left, forward, -turnCcw);
            writeTelemetry(null);
        }
        robot.getDrivetrain().stop();
        if (!reached) {
            RobotLog.w("[AUTO] %s stopped short: target X=%.1f Y=%.1f h=%.1f, at X=%.1f Y=%.1f h=%.1f",
                    step, tx, ty, th, odometry.getPosXMm() / 25.4, odometry.getPosYMm() / 25.4,
                    odometry.getHeadingDegrees());
        }
        return reached;
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updateRobot();
            writeTelemetry(null);
        }
    }

    private void setStep(String newStep) {
        step = newStep;
        RobotLog.i("[AUTO] %.1f s: %s", matchTimer.seconds(), step);
    }

    private double timeLeft() {
        return TunableValues.AUTO_LENGTH_S - matchTimer.seconds();
    }

    private static double redWallY() {
        return -(TunableValues.FIELD_SIZE_IN / 2.0 - TunableValues.ROBOT_SIZE_IN / 2.0);
    }

    private void writeTelemetry(List<AprilTagDetection> detections) {
        odometry.logOdometryDiagnostics();
        telemetry.addData("PROGRAM", program);
        telemetry.addData("STEP", step);
        telemetry.addData("TIME LEFT", "%.1f", timeLeft());
        telemetry.addData("FIELD X IN", "%.1f", odometry.getPosXMm() / 25.4);
        telemetry.addData("FIELD Y IN", "%.1f", odometry.getPosYMm() / 25.4);
        telemetry.addData("HEADING DEGREES", "%.1f", odometry.getHeadingDegrees());
        telemetry.addData("CAMERA FIX", cameraFix.getStatus());
        telemetry.addData("SHOOTER TICKS/S", "%.0f set / %.0f now",
                robot.getShooter().getTargetTps(), robot.getShooter().getMeasuredTps());
        if (detections != null) {
            addCellSeenTelemetry(detections);
        }
        telemetry.update();
    }

    /** Where the camera puts our cells if the odometry pose is right. Use to calibrate VISION_*_CELL_*. */
    private void addCellSeenTelemetry(List<AprilTagDetection> detections) {
        List<CellLocalizer.Sighting> seen = cells.sightings(detections, odometry.getHeadingDegrees());
        if (seen.isEmpty()) {
            telemetry.addData("CELL SEEN AT", "-");
            return;
        }
        double rx = odometry.getPosXMm() / 25.4;
        double ry = odometry.getPosYMm() / 25.4;
        for (CellLocalizer.Sighting s : seen) {
            telemetry.addData("CELL SEEN AT", "%s X %.1f Y %.1f", s.clusterName,
                    rx + s.fieldDxIn, ry + s.fieldDyIn);
        }
    }

    /** Once per loop: odometry, camera fixes, the shooter speed loop (and its follower), and the pose handoff. */
    private void updateRobot() {
        odometry.update();
        if (hopperStopMs >= 0 && hopperTimer.milliseconds() >= hopperStopMs) {
            robot.getHopper().stop();
            hopperStopMs = -1.0;
        }
        cameraFix.update(odometry, vision.sample());
        robot.getShooter().update();
        savePose();
    }

    private void savePose() {
        if (!odometry.isPresent()) {
            return;
        }
        MatchState.saveAutoPose(odometry.getPosXMm() / 25.4, odometry.getPosYMm() / 25.4,
                odometry.getHeadingDegrees());
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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
}
