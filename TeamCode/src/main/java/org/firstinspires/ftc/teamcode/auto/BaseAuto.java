package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

/**
 * Base Autonomous class that provides core functionality for all autonomous routines.
 * Handles subsystem initialization, common autonomous actions, path following, and utility methods.
 * All specific autonomous OpModes should extend this class.
 */
public abstract class BaseAuto extends OpMode {

    // ==================== SUBSYSTEM REFERENCES ====================
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected FlywheelSubsystem flywheelSubsystem;
    protected FeederSubsystem feederSubsystem;
    protected VisionSubsystem visionSubsystem;
    protected TurretSubsystem turretSubsystem;
    protected HoodSubsystem hoodSubsystem;

    // ==================== PATH FOLLOWING ====================
    protected Follower follower;

    // ==================== TIMERS ====================
    protected Timer opModeTimer;
    protected Timer pathTimer;
    protected Timer shootTimer;
    protected Timer actionTimer;

    // ==================== STATE TRACKING ====================
    protected boolean hasSpunUp = false;
    protected Alliance alliance;

    // ==================== TELEMETRY ====================
    protected TelemetryPacket dashboardPacket;

    /**
     * Initialize all subsystems and hardware.
     * Called automatically by FTC SDK during OpMode initialization.
     */
    @Override
    public void init() {
        // Set alliance (override this in child classes if needed)
        alliance = getAlliance();
        Robot.alliance = alliance;
        Robot.sendHardwareMap(hardwareMap);

        // Initialize telemetry
        initTelemetry();

        // Initialize path follower
        initFollower();

        // Initialize all timers
        initTimers();

        // Initialize all subsystems
        initSubsystems();

        // Build paths (implemented by child classes)
        buildPaths();

        // Set starting pose (implemented by child classes)
        setStartingPose();

        telemetry.addLine("✓ Initialization Complete");
        telemetry.update();
    }

    /**
     * Called when the OpMode starts (after init, when START is pressed).
     */
    @Override
    public void start() {
        // Start vision processing
        if (visionSubsystem != null) {
            visionSubsystem.start();
        }

        // Reset and start timers
        opModeTimer.resetTimer();
        shootTimer.resetTimer();
        pathTimer.resetTimer();

        // Initialize the first state
        initializeFirstState();

        telemetry.addLine("✓ Autonomous Started");
        telemetry.update();
    }

    /**
     * Main loop - updates all subsystems and handles state machine.
     */
    @Override
    public void loop() {
        // Update path follower
        follower.update();

        // Update vision
        if (visionSubsystem != null) {
            visionSubsystem.loop();
        }

        // Update state machine (implemented by child classes)
        updateStateMachine();

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Called when OpMode stops.
     */
    @Override
    public void stop() {
        // Save final pose for teleop transition
        Robot.lastPose = follower.getPose();

        // Stop all subsystems
        stopAllSubsystems();

        telemetry.addLine("✓ Autonomous Stopped");
        telemetry.update();
    }

    // ==================== INITIALIZATION METHODS ====================

    /**
     * Initialize telemetry with FTC Dashboard support.
     */
    protected void initTelemetry() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        dashboardPacket = new TelemetryPacket();
    }

    /**
     * Initialize the Pedro Pathing follower.
     */
    protected void initFollower() {
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Initialize all timers used in autonomous.
     */
    protected void initTimers() {
        opModeTimer = new Timer();
        pathTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
    }

    /**
     * Initialize all subsystems.
     * Gets singleton instances and calls init() on each.
     */
    protected void initSubsystems() {
        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        // Initialize Drive
        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        driveSubsystem.init();
        telemetry.addLine("  ✓ Drive");
        telemetry.update();

        // Initialize Intake
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem.init();
        telemetry.addLine("  ✓ Intake");
        telemetry.update();

        // Initialize Flywheel
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem.init();
        telemetry.addLine("  ✓ Flywheel");
        telemetry.update();

        // Initialize Hood
        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        hoodSubsystem.init();
        telemetry.addLine("  ✓ Hood");
        telemetry.update();

        // Initialize Feeder
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();
        telemetry.addLine("  ✓ Feeder");
        telemetry.update();

        // Initialize Vision
        visionSubsystem = VisionSubsystem.getInstance(hardwareMap, gamepad1);
        visionSubsystem.init();
        telemetry.addLine("  ✓ Vision");
        telemetry.update();

        // Initialize Turret
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        turretSubsystem.init();
        telemetry.addLine("  ✓ Turret");
        telemetry.update();
    }

    // ==================== COMMON AUTONOMOUS ACTIONS ====================

    /**
     * Shoot preloaded game pieces using field-relative calculations.
     * Uses ShotCalculator to determine flywheel velocity and hood angle based on distance.
     *
     * @return true if shooting sequence is complete
     */
    protected boolean shootPreload() {
        return shootSequence();
    }

    /**
     * Generic shooting sequence using field-relative calculations.
     * Automatically calculates flywheel velocity and hood angle based on distance to target.
     *
     * @return true if shooting sequence is complete
     */
    protected boolean shootSequence() {
        // Set velocity and hood angle based on distance
        flywheelSubsystem.setVelocityFromDistance();
        hoodSubsystem.setAngleFromDistance();

        // Aim turret
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        // Wait for flywheel to reach target velocity
        if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        // Start feeding when ready
        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
            intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
        }

        // Activate kicker after delay
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        // Complete shooting sequence
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            return true;
        }

        return false;
    }

    /**
     * Shoot with manual velocity override (for fixed positions like preload).
     * Still uses field-relative hood angle from ShotCalculator.
     *
     * @param targetVelocity Flywheel velocity setpoint (RPM)
     * @return true if shooting sequence is complete
     */
    protected boolean shootSequenceManual(double targetVelocity) {
        // Set manual velocity but use calculated hood angle
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngleFromDistance();

        // Aim turret
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        // Wait for flywheel to reach target velocity
        if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        // Start feeding when ready
        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
            intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
        }

        // Activate kicker after delay
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        // Complete shooting sequence
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            return true;
        }

        return false;
    }

    /**
     * Shoot with full manual control (for special cases).
     *
     * @param targetVelocity Flywheel velocity setpoint (RPM)
     * @param hoodAngle Hood angle in servo position (0.0 to 1.0)
     * @return true if shooting sequence is complete
     */
    protected boolean shootSequenceFullManual(double targetVelocity, double hoodAngle) {
        // Set manual velocity and hood angle
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngle(hoodAngle);

        // Aim turret
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        // Wait for flywheel to reach target velocity
        if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        // Start feeding when ready
        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
            intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
        }

        // Activate kicker after delay
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        // Complete shooting sequence
        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            return true;
        }

        return false;
    }

    /**
     * Run intake to collect game pieces.
     */
    protected void runIntake() {
        intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
        feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
    }

    /**
     * Stop intake and feeder.
     */
    protected void stopIntake() {
        intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);
        feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);
    }

    /**
     * Reverse intake to eject game pieces.
     */
    protected void ejectIntake() {
        intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.OUT);
        feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.OUT);
    }

    /**
     * Prepare robot for shooting using field-relative calculations (spin up, don't feed yet).
     * Automatically sets flywheel velocity and hood angle based on distance.
     */
    protected void prepareToShoot() {
        flywheelSubsystem.setVelocityFromDistance();
        hoodSubsystem.setAngleFromDistance();
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
    }

    /**
     * Prepare robot for shooting with manual velocity override (spin up, don't feed yet).
     * Uses field-relative hood angle calculation.
     *
     * @param targetVelocity Flywheel velocity setpoint (RPM)
     */
    protected void prepareToShootManual(double targetVelocity) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngleFromDistance();
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
    }

    /**
     * Prepare robot for shooting with full manual control (spin up, don't feed yet).
     *
     * @param targetVelocity Flywheel velocity setpoint (RPM)
     * @param hoodAngle Hood angle in servo position (0.0 to 1.0)
     */
    protected void prepareToShootFullManual(double targetVelocity, double hoodAngle) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngle(hoodAngle);
        turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());
    }

    /**
     * Set hood angle manually.
     *
     * @param angle Hood angle in servo position (0.0 to 1.0)
     */
    protected void setHoodAngle(double angle) {
        hoodSubsystem.setAngle(angle);
    }

    /**
     * Set hood angle based on distance to target (using ShotCalculator).
     */
    protected void setHoodAngleFromDistance() {
        hoodSubsystem.setAngleFromDistance();
    }

    /**
     * Reset hood to lowest angle (safe position).
     */
    protected void resetHood() {
        hoodSubsystem.setAngle(HoodConstants.LOWEST_ANGLE);
    }

    /**
     * Check if flywheel is at target velocity.
     *
     * @return true if at velocity
     */
    protected boolean isFlywheelReady() {
        return flywheelSubsystem.atVelocity();
    }

    /**
     * Reset the path state timer
     */
    protected void resetPathTimer() {
        pathTimer.resetTimer();
    }

    /**
     * Stop all subsystems and reset to safe state.
     */
    protected void stopAllSubsystems() {
        turretSubsystem.setPosition(0);
        hoodSubsystem.setAngle(HoodConstants.LOWEST_ANGLE);
        flywheelSubsystem.stop();
        feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);
        feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);
        intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);
    }

    // ==================== TELEMETRY METHODS ====================

    /**
     * Update telemetry with current robot state.
     * Override this to add custom telemetry in child classes.
     */
    protected void updateTelemetry() {

        // Add basic telemetry
        telemetry.addLine("========== TIMERS ==========");
        telemetry.addData("OpMode Timer", "%.2f s", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Timer", "%.2f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shoot Timer", "%.2f s", shootTimer.getElapsedTimeSeconds());

        telemetry.addLine("========== POSITION ==========");
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("========== STATUS ==========");
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Flywheel Ready", flywheelSubsystem.atVelocity());

        telemetry.update();
    }

    /**
     * Add state-specific telemetry.
     * Call this from child class updateTelemetry() after calling super.
     *
     * @param currentState The current state enum
     */
    protected void addStateTelemetry(Enum<?> currentState) {
        telemetry.addLine("========== STATE ==========");
        telemetry.addData("Current State", currentState.toString());
    }

    // ==================== ABSTRACT METHODS (Must be implemented by child classes) ====================

    /**
     * Get the alliance color for this autonomous routine.
     *
     * @return Alliance.BLUE or Alliance.RED
     */
    protected abstract Alliance getAlliance();

    /**
     * Build all paths for this autonomous routine.
     * This is where you create PathChains using the follower.pathBuilder().
     */
    protected abstract void buildPaths();

    /**
     * Set the starting pose for the robot.
     * Should call follower.setStartingPose(pose).
     */
    protected abstract void setStartingPose();

    /**
     * Initialize the first state of the autonomous routine.
     * This is called in start() after timers are reset.
     */
    protected abstract void initializeFirstState();

    /**
     * Update the state machine.
     * This is the main autonomous logic that runs every loop.
     */
    protected abstract void updateStateMachine();
}