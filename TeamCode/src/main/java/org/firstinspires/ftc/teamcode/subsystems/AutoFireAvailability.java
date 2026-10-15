package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
import org.firstinspires.ftc.teamcode.interfaces.AutoFireAvailabilityInterface;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;
import org.firstinspires.ftc.teamcode.interfaces.ShootingGeometryInterface;
import org.firstinspires.ftc.teamcode.interfaces.TargetCellTrackerInterface;
import org.firstinspires.ftc.teamcode.interfaces.VisionInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class AutoFireAvailability implements AutoFireAvailabilityInterface {
    private final RobotLogger log = new RobotLogger("AVAIL");
    private final AllianceInterface alliance;
    private final ImuInterface imu;
    private final VisionInterface vision;
    private final DrivetrainInterface drive;
    private final TargetCellTrackerInterface tracker;
    private final ShootingGeometryInterface shooting;
    private String lastReason = "";

    public AutoFireAvailability(AllianceInterface alliance, ImuInterface imu, VisionInterface vision,
                                DrivetrainInterface drive, TargetCellTrackerInterface tracker,
                                ShootingGeometryInterface shooting) {
        this.alliance = alliance;
        this.imu = imu;
        this.vision = vision;
        this.drive = drive;
        this.tracker = tracker;
        this.shooting = shooting;
    }

    @Override
    public boolean isAutoFireAvailable() {
        return unavailableReason() == null;
    }

    @Override
    public String unavailableReason() {
        String reason = computeReason();
        if (!reason.equals(lastReason)) {
            lastReason = reason;
            log.i("AUTO_FIRE_AVAILABLE=%b reason=%s", reason.isEmpty(), reason.isEmpty() ? "ok" : reason);
        }
        return reason.isEmpty() ? null : reason;
    }

    private String computeReason() {
        if (!alliance.isSet()) {
            return "alliance not set";
        }
        if (!imu.isPresent()) {
            return "no IMU";
        }
        if (!vision.isPresent()) {
            return "no webcam";
        }
        if (!drive.areDriveMotorsPresent()) {
            return "drive encoders/motors missing";
        }
        if (!tracker.isGatePassing()) {
            return "detection gate " + tracker.getGateHits() + "/" + tracker.getGateWindow();
        }
        CellGeometry geometry = tracker.getLastKnownGood();
        if (geometry == null) {
            return "no geometry";
        }
        if (!shooting.isInsideCone(geometry)) {
            return "outside cone";
        }
        double range = shooting.horizontalRangeIn(geometry);
        if (range > TunableValues.AUTO_FIRE_MAX_START_RANGE_IN) {
            return String.format("too far (%.1f in > %.1f)", range, TunableValues.AUTO_FIRE_MAX_START_RANGE_IN);
        }
        return "";
    }
}
