package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;

/**
 * Keeps odometry X/Y honest with the cell tags. Each new camera frame adds how far the
 * camera's robot position is from odometry's; every {@link TunableValues#VISION_FIX_INTERVAL_S}
 * the median of the recent offsets is applied. Offsets (not positions) are kept so readings
 * taken while driving still line up with where odometry is now.
 */
public class CameraPoseCorrector {

    private final CellLocalizer cells = new CellLocalizer();
    private final ElapsedTime clock = new ElapsedTime();
    private final ElapsedTime sinceFix = new ElapsedTime();
    /** {time s, dx in, dy in} */
    private final List<double[]> offsets = new ArrayList<>();
    private long lastFrameNanos = -1;
    private int fixCount;
    private double lastDxIn;
    private double lastDyIn;
    private boolean haveFix;

    /** Call once per loop after odometry.update(). Returns true when the pose was moved. */
    public boolean update(DeadWheelOdometry odometry, List<AprilTagDetection> detections) {
        if (!TunableValues.VISION_FIX_ENABLED || !odometry.isPresent()) {
            return false;
        }
        double now = clock.seconds();
        Iterator<double[]> it = offsets.iterator();
        while (it.hasNext()) {
            if (now - it.next()[0] > TunableValues.VISION_FIX_WINDOW_S) {
                it.remove();
            }
        }
        addSightings(odometry, detections, now);

        if (sinceFix.seconds() < TunableValues.VISION_FIX_INTERVAL_S
                || offsets.size() < TunableValues.VISION_FIX_MIN_SIGHTINGS) {
            return false;
        }
        double dx = median(0);
        double dy = median(1);
        double odoX = odometry.getPosXMm() / 25.4;
        double odoY = odometry.getPosYMm() / 25.4;
        odometry.setPositionInches(odoX + dx, odoY + dy);
        RobotLog.i("[CAMERA FIX] X %.1f -> %.1f, Y %.1f -> %.1f (%d sightings)",
                odoX, odoX + dx, odoY, odoY + dy, offsets.size());
        offsets.clear();
        sinceFix.reset();
        fixCount++;
        lastDxIn = dx;
        lastDyIn = dy;
        haveFix = true;
        return true;
    }

    /** Call after the pose is set some other way (wall square); held offsets no longer apply. */
    public void discardSightings() {
        offsets.clear();
    }

    /** One line for telemetry: the last correction and how long ago. */
    public String getStatus() {
        if (!TunableValues.VISION_FIX_ENABLED) {
            return "OFF";
        }
        if (!haveFix) {
            return String.format("none yet (%d sightings)", offsets.size());
        }
        return String.format("#%d moved X %+.1f Y %+.1f, %.0f s ago", fixCount, lastDxIn, lastDyIn,
                sinceFix.seconds());
    }

    private void addSightings(DeadWheelOdometry odometry, List<AprilTagDetection> detections, double now) {
        if (detections == null || detections.isEmpty()) {
            return;
        }
        long frame = detections.get(0).frameAcquisitionNanoTime;
        if (frame == lastFrameNanos) {
            return;
        }
        lastFrameNanos = frame;
        if (Math.abs(Math.toDegrees(odometry.getVelHeadingRadS())) > TunableValues.VISION_FIX_MAX_TURN_DEG_S) {
            return;
        }
        double odoX = odometry.getPosXMm() / 25.4;
        double odoY = odometry.getPosYMm() / 25.4;
        for (CellLocalizer.Sighting s : cells.sightings(detections, odometry.getHeadingDegrees())) {
            if (s.rangeIn > TunableValues.VISION_FIX_MAX_RANGE_IN) {
                continue;
            }
            double dx = s.robotXIn() - odoX;
            double dy = s.robotYIn() - odoY;
            if (Math.hypot(dx, dy) > TunableValues.VISION_FIX_MAX_JUMP_IN) {
                continue;
            }
            offsets.add(new double[] {now, dx, dy});
        }
    }

    private double median(int column) {
        List<Double> values = new ArrayList<>();
        for (double[] o : offsets) {
            values.add(o[column + 1]);
        }
        Collections.sort(values);
        return values.get(values.size() / 2);
    }
}
