package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.geometry.MovementPlan;

public interface ShootingGeometryInterface {
    boolean isInsideCone(CellGeometry geometry);

    boolean isInPosition(CellGeometry geometry);

    double horizontalRangeIn(CellGeometry geometry);

    double aimErrorDeg(CellGeometry geometry);

    MovementPlan planToArc(CellGeometry geometry);
}
