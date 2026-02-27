package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

/**
 * Base Autonomous class that provides core functionality for all autonomous routines.
 * Handles subsystem initialization, common autonomous actions, path following, and utility methods.
 * All specific autonomous OpModes should extend this class.
 */
public abstract class BaseAuto extends BaseOpMode {

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
    protected boolean flywheelShouldNotIdle = false;
    protected boolean shouldVisionUpdate = true;
    protected boolean isShooting = false;

    // ==================== TELEMETRY ====================
    protected TelemetryPacket dashboardPacket;

    // ==================== LIFECYCLE HOOKS ====================
    @Override
    protected void onInit() {
        Robot.sendHardwareMap(hardwareMap);
        Robot.alliance = getAlliance();

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

    @Override
    protected void onStart() {
        // Start vision processing
        if (visionSubsystem != null) visionSubsystem.start();

        opModeTimer.resetTimer();
        shootTimer.resetTimer();
        pathTimer.resetTimer();

        initializeFirstState();

        telemetry.addLine("✓ Autonomous Started");
        telemetry.update();
    }

    @Override
    protected void onLoop() {
        // Update path follower
        follower.update();

        // Update vision
        if (visionSubsystem != null) visionSubsystem.autoLoop(isShooting);

        turretSubsystem.loop();

        if (!flywheelShouldNotIdle) flywheelSubsystem.setPower(.35);

        updateStateMachine();
        updateTelemetry();
    }

    @Override
    protected void onStop() {
        Robot.lastPose = follower.getPose();
        stopAllSubsystems();

        telemetry.addLine("✓ Autonomous Stopped");
        telemetry.update();
    }

    // ==================== INITIALIZATION METHODS ====================

    protected void initTelemetry() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        dashboardPacket = new TelemetryPacket();
    }

    protected void initFollower() {
        follower = Constants.createFollower(hardwareMap);
    }

    protected void initTimers() {
        opModeTimer = new Timer();
        pathTimer = new Timer();
        shootTimer = new Timer();
        actionTimer = new Timer();
    }

    protected void initSubsystems() {
        telemetry.addLine("Initializing Subsystems...");
        telemetry.update();

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        driveSubsystem.autoInit(follower);
        telemetry.addLine("  ✓ Drive");
        telemetry.update();

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem.init();
        telemetry.addLine("  ✓ Intake");
        telemetry.update();

        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem.init();
        telemetry.addLine("  ✓ Flywheel");
        telemetry.update();

        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        hoodSubsystem.init();
        telemetry.addLine("  ✓ Hood");
        telemetry.update();

        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem.init();
        telemetry.addLine("  ✓ Feeder");
        telemetry.update();

        visionSubsystem = VisionSubsystem.getInstance(hardwareMap, gamepad1);
        visionSubsystem.init();
        telemetry.addLine("  ✓ Vision");
        telemetry.update();

        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        turretSubsystem.init();
        telemetry.addLine("  ✓ Turret");
        telemetry.update();
    }

    // ==================== COMMON AUTONOMOUS ACTIONS ====================

    protected boolean shootSequence() {
        flywheelShouldNotIdle = true;
        shouldVisionUpdate = false;
        stopIntake();
        revFlywheel();
        hoodSubsystem.setAngleFromDistanceWithCorrection();

        turretSubsystem.setFieldRelativeAngle();
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        if (flywheelSubsystem.atVelocity() && !hasSpunUp && follower.getVelocity().getMagnitude() < 1) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.feederState.IN);
            intakeSubsystem.setState(IntakeConstants.intakeState.INTAKE);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 1.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 2.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            flywheelShouldNotIdle = false;
            isShooting = false;
            shouldVisionUpdate = true;
            return true;
        }

        return false;
    }

    protected void revFlywheel() {
        flywheelShouldNotIdle = true;
        flywheelSubsystem.setVelocityFromDistance();
    }

    protected boolean shootSequenceManual(double targetVelocity) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngleFromDistanceWithCorrection();

        turretSubsystem.setFieldRelativeAngle();
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.feederState.IN);
            intakeSubsystem.setState(IntakeConstants.intakeState.INTAKE);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            return true;
        }

        return false;
    }

    protected boolean shootSequenceFullManual(double targetVelocity, double hoodAngle) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngle(hoodAngle);

        turretSubsystem.setFieldRelativeAngle();
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

        if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
            hasSpunUp = true;
            shootTimer.resetTimer();
        }

        if (hasSpunUp) {
            feederSubsystem.setFeederState(FeederConstants.feederState.IN);
            intakeSubsystem.setState(IntakeConstants.intakeState.INTAKE);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
        }

        if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
            feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
            feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
            flywheelSubsystem.setPower(0.35);
            hasSpunUp = false;
            return true;
        }

        return false;
    }

    protected void setFeederAndIntakeState(IntakeConstants.intakeState intakeState, FeederConstants.feederState feederState) {
        intakeSubsystem.setState(intakeState);
        feederSubsystem.setFeederState(feederState);
    }

    protected void stopIntake() {
        intakeSubsystem.setState(IntakeConstants.intakeState.STOP);
        feederSubsystem.setFeederState(FeederConstants.feederState.STOP);
    }

    protected void ejectIntake() {
        intakeSubsystem.setState(IntakeConstants.intakeState.OUT);
        feederSubsystem.setFeederState(FeederConstants.feederState.OUT);
    }

    protected void prepareToShoot() {
        flywheelSubsystem.setVelocityFromDistance();
        hoodSubsystem.setAngleFromDistanceWithCorrection();
        turretSubsystem.setFieldRelativeAngle();
    }

    protected void prepareToShootManual(double targetVelocity) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngleFromDistanceWithCorrection();
        turretSubsystem.setFieldRelativeAngle();
    }

    protected void prepareToShootFullManual(double targetVelocity, double hoodAngle) {
        flywheelSubsystem.setVelocity(targetVelocity);
        hoodSubsystem.setAngle(hoodAngle);
        turretSubsystem.setFieldRelativeAngle();
    }

    protected void setHoodAngle(double angle) {
        hoodSubsystem.setAngle(angle);
    }

    protected void setHoodAngleFromDistance() {
        hoodSubsystem.setAngleFromDistanceWithCorrection();
    }

    protected void resetHood() {
        hoodSubsystem.setAngle(HoodConstants.LOWEST_ANGLE);
    }

    protected boolean isFlywheelReady() {
        return flywheelSubsystem.atVelocity();
    }

    protected void resetPathTimer() {
        pathTimer.resetTimer();
    }

    protected void stopAllSubsystems() {
        flywheelShouldNotIdle = true;
        turretSubsystem.setPosition(0);
        hoodSubsystem.setAngle(HoodConstants.LOWEST_ANGLE);
        flywheelSubsystem.stop();
        feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
        feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);
        feederSubsystem.setFeederState(FeederConstants.feederState.STOP);
        intakeSubsystem.setState(IntakeConstants.intakeState.STOP);
    }

    // ==================== TELEMETRY METHODS ====================

    protected void updateTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        driveSubsystem.setTelemetry(packet);
//        turretSubsystem.setTelemetry(packet);
//        flywheelSubsystem.setTelemetry(packet);
//        hoodSubsystem.setTelemetry(packet);
        visionSubsystem.setTelemetry(packet);
//        intakeSubsystem.setTelemetry(packet);
//        feederSubsystem.setTelemetry(packet);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

//        telemetry.addLine("========== TIMERS ==========");
//        telemetry.addData("OpMode Timer", "%.2f s", opModeTimer.getElapsedTimeSeconds());
//        telemetry.addData("Path Timer", "%.2f s", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("Shoot Timer", "%.2f s", shootTimer.getElapsedTimeSeconds());
//
//        telemetry.addLine("========== POSITION ==========");
//        telemetry.addData("X", "%.2f", follower.getPose().getX());
//        telemetry.addData("Y", "%.2f", follower.getPose().getY());
//        telemetry.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
//
//        telemetry.addLine("========== STATUS ==========");
//        telemetry.addData("Follower Busy", follower.isBusy());
//        telemetry.addData("Flywheel Ready", flywheelSubsystem.atVelocity());
//
//        telemetry.update();
    }

    // ==================== ABSTRACT METHODS ====================
    protected abstract Alliance getAlliance();
    protected abstract void buildPaths();
    protected abstract void setStartingPose();
    protected abstract void initializeFirstState();
    protected abstract void updateStateMachine();
}
