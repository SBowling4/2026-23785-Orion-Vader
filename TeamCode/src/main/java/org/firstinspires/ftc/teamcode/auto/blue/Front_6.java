package org.firstinspires.ftc.teamcode.auto.blue;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.pedroPathing.Constants;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

@Autonomous(name = "Front_6_Blue")
public class Front_6 extends OpMode {
    private Follower follower;

    private enum PathState {
        DRIVE_SHOOT_POS,
        SHOOT_PRELOAD,
        PICKUP,
        DRIVE_PICKUP_SHOOT,
        SHOOT_PICKUP,
        OFFLINE,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(17.769911504424773, 121.41592920353982, Units.degreesToRadians(144));
    private final Pose shootPose = new Pose(52.3362831858407, 91.78761061946905, Units.degreesToRadians(144));
    private final Pose pickupPose = new Pose(16.490974729241874, 84.24548736462093, Units.degreesToRadians(180));
    private final Pose offlinePose = new Pose(44.3716814159292, 44.3716814159292, Units.degreesToRadians(180));

    private PathChain driveStartToShoot, drivePickup, drivePickupToShoot, driveOffline;
    Timer opModeTimer, pathTimer, shootTimer;

    private boolean hasSpunUp = false;

    private TurretSubsystem turretSubsystem;
    private DriveSubsystem driveSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private FeederSubsystem feederSubsystem;
    private Vision vision;

    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;
        Robot.sendHardwareMap(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        follower = Constants.createFollower(hardwareMap);

        pathState = PathState.DRIVE_SHOOT_POS;

        buildPaths();

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap, gamepad1);
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);

        opModeTimer = new Timer();
        pathTimer = new Timer();
        shootTimer = new Timer();


        vision.init();
        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
//        hoodSubsystem.init();
        feederSubsystem.init();
        turretSubsystem.init();

        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        vision.start();

        opModeTimer.resetTimer();
        shootTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();


        vision.loop();

        statePathUpdate();

        TelemetryPacket packet = new TelemetryPacket();

        driveSubsystem.setTelemetry(packet);
        turretSubsystem.setTelemetry(packet);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("State", pathState);
        telemetry.addLine("----------");
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shoot Timer", shootTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opModeTimer.getElapsedTimeSeconds());
        telemetry.addLine("----------");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
    }

    @Override
    public void stop() {
        Robot.lastPose = follower.getPose();
    }

    private void statePathUpdate() {
        switch (pathState) {
            case DRIVE_SHOOT_POS:
                follower.followPath(driveStartToShoot);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity());
                turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());

                feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

                if (flywheelSubsystem.atVelocity() && !hasSpunUp && !follower.isBusy()) {
                    hasSpunUp = true;
                    shootTimer.resetTimer();
                }

                if (hasSpunUp) {
                    feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
                    intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
                }

                if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
                    feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
                }

                if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
                    feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
                    feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
                    flywheelSubsystem.setPower(.35);

                    hasSpunUp = false;

                    follower.followPath(drivePickup);
                    setPathState(PathState.PICKUP);
                }
                break;
            case PICKUP:
                if (!follower.isBusy()) {
                    follower.followPath(drivePickupToShoot);
                    setPathState(PathState.DRIVE_PICKUP_SHOOT);
                }
                break;
            case DRIVE_PICKUP_SHOOT:
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);

                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity());
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP);
                }
                break;
            case SHOOT_PICKUP:
                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity());
                turretSubsystem.setPosition(turretSubsystem.findFieldRelativeAngle());

                feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

                if (flywheelSubsystem.atVelocity() && !hasSpunUp && !follower.isBusy()) {
                    hasSpunUp = true;
                    shootTimer.resetTimer();
                }

                if (hasSpunUp) {
                    feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);
                    intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
                }

                if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 3.5) {
                    feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.IN);
                }

                if (hasSpunUp && shootTimer.getElapsedTimeSeconds() > 4.5) {
                    feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
                    feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.CLOSED);
                    flywheelSubsystem.setPower(.35);

                    hasSpunUp = false;

                    follower.followPath(driveOffline);
                    setPathState(PathState.OFFLINE);
                }
                break;
            case OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
            case END:
                flywheelSubsystem.stop();
                feederSubsystem.setKickerState(FeederConstants.KICKER_STATE.OUT);
                feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);
                break;
        }
    }

    private void setPathState(PathState state) {
        pathState = state;
        pathTimer.resetTimer();

        if (state == PathState.SHOOT_PRELOAD || state == PathState.SHOOT_PICKUP) {
            hasSpunUp = false;
            shootTimer.resetTimer();
        }
    }

    public void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        drivePickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading(), .3)
                .build();

        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        driveOffline = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, offlinePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), offlinePose.getHeading())
                .build();
    }

}
