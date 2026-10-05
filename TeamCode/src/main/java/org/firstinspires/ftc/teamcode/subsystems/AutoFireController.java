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
 * the non-blocking states: IDLE -> ALIGNING -> SPINNING_UP -> FEEDING -> COMPLETE.
 */
public class AutoFireController implements AutoFireControllerInterface {

    // Set operational power multiplier for shooter flywheel motor during firing
    public static double SHOOTER_SPINUP_POWER = 1.0;
    // Set operational power multiplier for hopper feeder motor during firing
    public static double HOPPER_FEED_POWER    = 1.0;
    // Set delay in seconds to ensure flywheel reaches full launching RPM
    public static double SPINUP_DELAY_SECONDS = 0.5;
    // Set duration in seconds to feed all loaded game elements into shooter
    public static double FEED_DURATION_SECONDS = 2.0;

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
                // Reset state timer for alignment phase
                stateTimer.reset();
                // Transition state to ALIGNING (aiming & positioning)
                currentState = FiringState.ALIGNING;
                // Start spinning up shooter flywheel motor in background
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }
                break;

            case ALIGNING: // Aiming and positioning robot relative to target
                // Keep shooter flywheel spinning while aligning
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }

                // Query vision subsystem for spatial geometry of best target
                TargetGeometryInterface target = (vision != null) ? vision.getBestTargetGeometry() : null;

                // Flag tracking whether robot is in position
                boolean aligned;
                if (target != null && drivetrain != null) { // If valid target tag is acquired
                    // Auto-steer drivetrain to 24 inch distance and 0 degree heading
                    aligned = drivetrain.alignToTarget(target);
                } else { // Fallback if no target is visible
                    // Flag alignment ready
                    aligned = true;
                    if (drivetrain != null) {
                        drivetrain.stop();
                    }
                }

                // If drivetrain alignment is complete (robot is in position)
                if (aligned) {
                    // Stop drivetrain motors
                    if (drivetrain != null) {
                        drivetrain.stop();
                    }
                    // Reset state timer for flywheel spinup check
                    stateTimer.reset();
                    // Transition state to SPINNING_UP
                    currentState = FiringState.SPINNING_UP;
                }
                break;

            case SPINNING_UP: // Verifying shooter flywheel motor speed
                // Keep shooter flywheel motor spinning at 100 percent power
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }

                // Check if spinup delay time has elapsed
                if (stateTimer.seconds() >= SPINUP_DELAY_SECONDS) {
                    // Reset state timer for feeding phase
                    stateTimer.reset();
                    // Transition state to FEEDING
                    currentState = FiringState.FEEDING;
                }
                break;

            case FEEDING: // Releasing hopper and feeding elements into flywheel
                // Keep shooter flywheel motor spinning at full power
                if (shooter != null) {
                    shooter.runShooter(SHOOTER_SPINUP_POWER);
                }

                // Actuate hopper feeder motor to push elements into spinning flywheel
                if (hopper != null) {
                    hopper.runHopper(HOPPER_FEED_POWER);
                }

                // Check if feed duration timer has elapsed
                if (stateTimer.seconds() >= FEED_DURATION_SECONDS) {
                    // Transition state to COMPLETE
                    currentState = FiringState.COMPLETE;
                }
                break;

            case COMPLETE: // Firing sequence finished
                // Stop hopper feeder motor
                if (hopper != null) {
                    hopper.stop();
                }
                // Stop shooter flywheel motor
                if (shooter != null) {
                    shooter.stop();
                }
                // Reset state to IDLE
                currentState = FiringState.IDLE;
                break;

            case ABORTED: // Sequence interrupted
                // Execute safety abort
                abort();
                break;
        }
    }

    // Immediately halts all firing motors and returns state machine to IDLE
    @Override
    public void abort() {
        // Stop drivetrain motors
        if (drivetrain != null) {
            drivetrain.stop();
        }
        // Stop hopper feeder motor
        if (hopper != null) {
            hopper.stop();
        }
        // Stop shooter flywheel motor
        if (shooter != null) {
            shooter.stop();
        }
        // Reset state machine to IDLE
        currentState = FiringState.IDLE;
        // Reset state timer
        stateTimer.reset();
    }
}
