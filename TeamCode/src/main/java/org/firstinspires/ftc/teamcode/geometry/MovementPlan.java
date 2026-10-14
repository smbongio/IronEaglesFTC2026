package org.firstinspires.ftc.teamcode.geometry;

/**
 * One measure-then-move plan. Distances are in the robot frame at the moment of the measurement.
 * Drive forward is +Y, strafe right is +X, turn is degrees (positive = counter-clockwise, IMU convention).
 */
public final class MovementPlan {
    public final double driveInches;
    public final double strafeInches;
    public final double turnDegrees;

    public MovementPlan(double driveInches, double strafeInches, double turnDegrees) {
        this.driveInches = driveInches;
        this.strafeInches = strafeInches;
        this.turnDegrees = turnDegrees;
    }

    @Override
    public String toString() {
        return String.format("drive=%.1f in  strafe=%.1f in  turn=%.1f deg", driveInches, strafeInches, turnDegrees);
    }
}
