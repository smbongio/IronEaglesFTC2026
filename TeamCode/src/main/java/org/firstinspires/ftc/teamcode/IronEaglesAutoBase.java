package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.subsystems.DeadWheelOdometry;

/**
 * Shared Autonomous for both alliances. No driving yet: the robot holds still and
 * tracks its pose. Records the alliance at INIT and the final pose at the end so
 * TeleOp starts on the right alliance at the right place.
 */
public abstract class IronEaglesAutoBase extends LinearOpMode {

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() {
        Alliance side = alliance();
        RobotLog.i("OpMode Initializing: IronEagles Auto %s", side);

        IronEaglesHardware robot = new IronEaglesHardware();
        robot.init(hardwareMap);
        DeadWheelOdometry odometry = new DeadWheelOdometry(robot.leftPod, robot.rightPod, robot.rearPod,
                side.startXIn(), side.startYIn(), side.startHeadingDeg());
        MatchState.alliance = side;
        MatchState.clearAutoPose();
        robot.getLed().setAlliance(side);

        telemetry.addData("ALLIANCE", side);
        telemetry.addData("START POSE", "X %.1f Y %.1f heading %.1f",
                side.startXIn(), side.startYIn(), side.startHeadingDeg());
        if (!robot.missingDevices.isEmpty()) {
            telemetry.addData("MISSING", String.join(", ", robot.missingDevices));
        }
        telemetry.update();

        waitForStart();
        try {
            while (opModeIsActive()) {
                odometry.update();
                savePose(odometry);
                odometry.logOdometryDiagnostics();
                telemetry.addData("ALLIANCE", side);
                telemetry.addData("FIELD X IN", "%.1f", odometry.getPosXMm() / 25.4);
                telemetry.addData("FIELD Y IN", "%.1f", odometry.getPosYMm() / 25.4);
                telemetry.addData("HEADING DEGREES", "%.1f", odometry.getHeadingDegrees());
                telemetry.update();
            }
        } finally {
            if (odometry.isPresent() && MatchState.hasAutoPose) {
                RobotLog.i("[ALLIANCE] Auto %s ended at FIELD X=%.1f Y=%.1f heading=%.1f",
                        side, MatchState.autoXIn, MatchState.autoYIn, MatchState.autoHeadingDeg);
            }
            robot.getLed().close();
        }
    }

    private static void savePose(DeadWheelOdometry odometry) {
        if (!odometry.isPresent()) {
            return;
        }
        MatchState.saveAutoPose(odometry.getPosXMm() / 25.4, odometry.getPosYMm() / 25.4,
                odometry.getHeadingDegrees());
    }
}
