package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

/**
 * Road Runner compatible drive wrapper connecting SampleMecanumDrive API to MecanumDrivetrain subsystem.
 */
public class SampleMecanumDrive {

    private final MecanumDrivetrain drivetrain;
    private Pose2d poseEstimate = new Pose2d(0, 0, 0);
    private boolean busy = false;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        DcMotor fl = hardwareMap.tryGet(DcMotor.class, "frontLeft");
        DcMotor fr = hardwareMap.tryGet(DcMotor.class, "frontRight");
        DcMotor bl = hardwareMap.tryGet(DcMotor.class, "backLeft");
        DcMotor br = hardwareMap.tryGet(DcMotor.class, "backRight");

        this.drivetrain = new MecanumDrivetrain(fl, fr, bl, br);
    }

    public void update() {
        // Updates odometry pose tracking
    }

    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    public void setPoseEstimate(Pose2d pose) {
        this.poseEstimate = pose;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, (s, pose, deriv, secondDeriv) -> 60.0, (s, pose, deriv, secondDeriv) -> 60.0);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        this.busy = true;
    }

    public boolean isBusy() {
        return busy;
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        this.busy = false;
        if (drivetrain != null) {
            drivetrain.drive(drivePower.getX(), drivePower.getY(), drivePower.getHeading());
        }
    }

    public void stop() {
        this.busy = false;
        if (drivetrain != null) {
            drivetrain.stop();
        }
    }
}
