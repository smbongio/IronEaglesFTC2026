package org.firstinspires.ftc.teamcode.subsystems;

// Import FTC utility timer for non-blocking state duration checks
import com.qualcomm.robotcore.util.ElapsedTime;
// Import high-level auto-fire controller interface contract
import org.firstinspires.ftc.teamcode.interfaces.AutoFireControllerInterface;
// Import drivetrain subsystem interface
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
// Import hopper subsystem interface
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
// Import shooter subsystem interface
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;
// Import target spatial geometry interface
import org.firstinspires.ftc.teamcode.interfaces.TargetGeometryInterface;
// Import vision subsystem interface
import org.firstinspires.ftc.teamcode.interfaces.VisionSubsystemInterface;

/**
 * Concrete implementation of the automated firing sequence controller.
 * Orchestrates Vision, Drivetrain, Shooter, and Hopper subsystems through
 * the non-blocking states: IDLE -> ALIGNING -> FEEDING.
 */
public class AutoFireController implements AutoFireControllerInterface {

    // Set operational power multiplier for shooter flywheel motor during firing
    public static double SHOOTER_SPINUP_POWER = 1.0;
    // Set operational power multiplier for hopper feeder motor during firing
    public static double HOPPER_FEED_POWER    = 1.0;

    // Reference for drivetrain subsystem interface
    private final DrivetrainInterface drivetrain;
    // Reference for shooter subsystem interface
    private final ShooterInterface shooter;
    // Reference for hopper subsystem interface
    private final HopperInterface hopper;
    // Reference for vision subsystem interface
    private VisionSubsystemInterface vision;

    // Current state of automated firing sequence
    private FiringState currentState = FiringState.IDLE;
    // Elapsed timer for non-blocking state duration management
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Constructor assigning drivetrain, shooter, hopper, and vision subsystems
    public AutoFireController(DrivetrainInterface drivetrain,
                              ShooterInterface shooter,
                              HopperInterface hopper,
                              VisionSubsystemInterface vision) {
        this.drivetrain = drivetrain;
        this.shooter    = shooter;
        this.hopper     = hopper;
        this.vision     = vision;
    }

    // Constructor assigning drivetrain, shooter, and hopper subsystems
    public AutoFireController(DrivetrainInterface drivetrain,
                              ShooterInterface shooter,
                              HopperInterface hopper) {
        this(drivetrain, shooter, hopper, null);
    }

    // Assigns or updates reference for vision subsystem interface
    public void setVisionSubsystem(VisionSubsystemInterface vision) {
        this.vision = vision;
    }

    // Returns current state of automated firing sequence
    @Override
    public FiringState getCurrentState() {
        return currentState;
    }

    // Updates and advances non-blocking state machine loop cycle
    @Override
    public void update(boolean isFireTriggerHeld, boolean isDriverSteering) {
        // If trigger is released or driver is manually steering joysticks
        if (!isFireTriggerHeld || isDriverSteering) { // Abort sequence immediately
            if (currentState != FiringState.IDLE) {
                abort();
            }
            return;
        }

        // Advance non-blocking state machine based on current state
        switch (currentState) {

            case IDLE: // Waiting to start automated sequence
                stateTimer.reset();
                currentState = FiringState.ALIGNING;
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }
                break;

            case ALIGNING: // Aiming and positioning robot relative to target
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }

                TargetGeometryInterface target = (vision != null) ? vision.getBestTargetGeometry() : null;

                boolean aligned;
                if (target != null && drivetrain != null) { // If valid target tag is acquired
                    aligned = drivetrain.alignToTarget(target);
                } else { // Fallback if no target is visible
                    aligned = true;
                    if (drivetrain != null) {
                        drivetrain.stop();
                    }
                }

                // If drivetrain alignment is complete (robot is in position)
                if (aligned) {
                    if (drivetrain != null) {
                        drivetrain.stop();
                    }
                    stateTimer.reset();
                    currentState = FiringState.FEEDING;
                }
                break;

            case FEEDING: // Releasing hopper and feeding elements into flywheel
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }
                if (hopper != null) {
                    hopper.runHopper(HOPPER_FEED_POWER);
                }
                break;
        }
    }

    // Immediately halts all firing motors and returns state machine to IDLE
    @Override
    public void abort() {
        if (drivetrain != null) {
            drivetrain.stop();
        }
        if (hopper != null) {
            hopper.stop();
        }
        if (shooter != null) {
            shooter.stop();
        }
        currentState = FiringState.IDLE;
        stateTimer.reset();
    }
}
