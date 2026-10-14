package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.geometry.MovementPlan;
import org.firstinspires.ftc.teamcode.interfaces.ShootingGeometryInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class ShootingGeometry implements ShootingGeometryInterface {
    private final RobotLogger log = new RobotLogger("GEOMETRY");
    private String lastConeKey = "";
    private String lastPositionKey = "";

    @Override
    public boolean isInsideCone(CellGeometry geometry) {
        if (geometry == null) {
            return false;
        }
        double facingX = Math.sin(Math.toRadians(geometry.yawDeg));
        double facingY = -Math.cos(Math.toRadians(geometry.yawDeg));
        double toRobotX = -geometry.xIn;
        double toRobotY = -geometry.yIn;
        double toRobotLen = Math.hypot(toRobotX, toRobotY);
        if (toRobotLen < 0.1) {
            return false;
        }
        double dot = (facingX * toRobotX + facingY * toRobotY) / toRobotLen;
        double angle = Math.toDegrees(Math.acos(Math.max(-1.0, Math.min(1.0, dot))));
        boolean inside = Math.abs(angle) <= 45.0;
        String key = String.format("%b|%.0f|%.0f", inside, angle, geometry.rangeIn);
        if (!key.equals(lastConeKey)) {
            lastConeKey = key;
            log.i("cone angle=%.1f deg inside=%b range=%.1f yaw=%.1f", angle, inside, geometry.rangeIn, geometry.yawDeg);
        }
        return inside;
    }

    @Override
    public boolean isInPosition(CellGeometry geometry) {
        if (geometry == null || !isInsideCone(geometry)) {
            return false;
        }
        double rangeError = Math.abs(horizontalRangeIn(geometry) - TunableValues.SHOOTING_DISTANCE_IN);
        double aim = Math.abs(aimErrorDeg(geometry));
        boolean inPosition = rangeError <= TunableValues.DISTANCE_TOLERANCE_IN
                && aim <= TunableValues.ALIGNMENT_TOLERANCE_DEG;
        String key = String.format("%b|%.1f|%.1f", inPosition, rangeError, aim);
        if (!key.equals(lastPositionKey)) {
            lastPositionKey = key;
            log.i("inPosition=%b rangeError=%.1f in aimError=%.1f deg", inPosition, rangeError, aim);
        }
        return inPosition;
    }

    @Override
    public double horizontalRangeIn(CellGeometry geometry) {
        return geometry == null ? 0.0 : Math.hypot(geometry.xIn, geometry.yIn);
    }

    @Override
    public double aimErrorDeg(CellGeometry geometry) {
        return geometry == null ? 0.0 : geometry.bearingDeg + TunableValues.AIM_OFFSET_DEG;
    }

    @Override
    public MovementPlan planToArc(CellGeometry geometry) {
        double horiz = horizontalRangeIn(geometry);
        double along = horiz - TunableValues.SHOOTING_DISTANCE_IN;
        double unitX = horiz < 0.1 ? 0.0 : geometry.xIn / horiz;
        double unitY = horiz < 0.1 ? 1.0 : geometry.yIn / horiz;
        MovementPlan plan = new MovementPlan(unitY * along, unitX * along, aimErrorDeg(geometry));
        log.i("plan %s (horiz=%.1f along=%.1f)", plan, horiz, along);
        return plan;
    }
}
