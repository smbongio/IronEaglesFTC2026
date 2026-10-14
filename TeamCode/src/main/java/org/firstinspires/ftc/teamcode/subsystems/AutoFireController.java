package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.geometry.MovementPlan;
import org.firstinspires.ftc.teamcode.interfaces.AutoFireAvailabilityInterface;
import org.firstinspires.ftc.teamcode.interfaces.AutoFireControllerInterface;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
import org.firstinspires.ftc.teamcode.interfaces.ShootingGeometryInterface;
import org.firstinspires.ftc.teamcode.interfaces.TargetCellTrackerInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class AutoFireController implements AutoFireControllerInterface {
    private final RobotLogger log = new RobotLogger("AUTOFIRE");
    private final AutoFireAvailabilityInterface availability;
    private final TargetCellTrackerInterface tracker;
    private final ShootingGeometryInterface shooting;
    private final DrivetrainInterface drive;
    private final HopperInterface hopper;
    private State state = State.IDLE;

    public AutoFireController(AutoFireAvailabilityInterface availability, TargetCellTrackerInterface tracker,
                              ShootingGeometryInterface shooting, DrivetrainInterface drive, HopperInterface hopper) {
        this.availability = availability;
        this.tracker = tracker;
        this.shooting = shooting;
        this.drive = drive;
        this.hopper = hopper;
    }

    @Override
    public void start() {
        if (state != State.IDLE) {
            log.i("start ignored; already %s", state);
            return;
        }
        if (!availability.isAutoFireAvailable()) {
            log.w("start ignored; %s", availability.unavailableReason());
            return;
        }
        drive.stop();
        setState(State.MEASURING);
    }

    @Override
    public void abort() {
        if (state == State.IDLE) {
            return;
        }
        log.i("abort from %s", state);
        hopper.stop();
        setState(State.IDLE);
    }

    @Override
    public void update() {
        if (state == State.IDLE) {
            return;
        }
        if (state == State.MOVING) {
            if (drive.isMoveFinished()) {
                setState(State.MEASURING);
            }
            return;
        }
        if (!availability.isAutoFireAvailable()) {
            log.w("ended: %s", availability.unavailableReason());
            hopper.stop();
            drive.stop();
            setState(State.IDLE);
            return;
        }
        CellGeometry geometry = tracker.getLastKnownGood();
        if (state == State.MEASURING) {
            if (shooting.isInPosition(geometry)) {
                drive.stop();
                setState(State.FEEDING);
            } else {
                MovementPlan plan = shooting.planToArc(geometry);
                drive.startMove(plan);
                setState(State.MOVING);
            }
            return;
        }
        if (state == State.FEEDING) {
            hopper.stepForward180();
            hopper.update();
        }
    }

    @Override
    public State getState() {
        return state;
    }

    @Override
    public boolean isActive() {
        return state != State.IDLE;
    }

    private void setState(State next) {
        if (state != next) {
            log.i("%s -> %s", state, next);
            state = next;
        }
    }
}
