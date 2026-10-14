package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public interface TargetCellTrackerInterface {
    void setAlliance(AllianceInterface.Color color);

    void update(List<AprilTagDetection> detections);

    boolean isTargetCellInView();

    boolean isGatePassing();

    int getGateHits();

    int getGateWindow();

    CellGeometry getLastKnownGood();
}
