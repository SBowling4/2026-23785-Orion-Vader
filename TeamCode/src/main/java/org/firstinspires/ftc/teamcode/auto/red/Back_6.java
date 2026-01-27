package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
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
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

@Autonomous(name = "Back_6_HP_Red")
public class Back_6 extends OpMode {
    private Follower follower;

    private enum PathState {
        SHOOT_PRELOAD,
        DRIVE_PICKUP,
        WIGGLE_BACK,
        WIGGLE_FORWARD,
        DRIVE_PICKUP_TO_SHOOT,
        SHOOT_PICKUP,
        OFFLINE,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(44.014440433213, 9.184115523465712, Units.degreesToRadians(180)).mirror();
    private final Pose pickupPose = new Pose(9.010830324909747, 8.144404332129957, Units.degreesToRadians(180)).mirror();
    private final Pose wigglePose = new Pose(21.140794223826713, 8.837545126353788, Math.toRadians(180)).mirror();
    private final Pose shootPose = new Pose(48.51985559566786, 8.49097472924188, Units.degreesToRadians(180)).mirror();
    private final Pose offlinePose = new Pose(48.69314079422383, 34.65703971119134, Units.degreesToRadians(180)).mirror();

    private PathChain driveToPickup, driveToShoot, driveToOffline, wiggleBack, wiggleForward;

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
        Robot.alliance = Alliance.RED;
        Robot.sendHardwareMap(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        follower = Constants.createFollower(hardwareMap);

        pathState = PathState.SHOOT_PRELOAD;

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

        buildPaths();
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
            case SHOOT_PRELOAD:
                flywheelSubsystem.setVelocity(FlywheelConstants.FAR_SP);
                turretSubsystem.setPosition(TurretConstants.RED_FAR_SP);

                feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

                if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
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

                    follower.followPath(driveToPickup, .75, false);
                    setPathState(PathState.DRIVE_PICKUP);
                }
                break;
            case DRIVE_PICKUP:
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(wiggleBack, .5, false);
                    setPathState(PathState.WIGGLE_BACK);
                }
                break;
            case WIGGLE_BACK:
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);

                if (!follower.isBusy()) {
                    follower.followPath(wiggleForward, .5, false);
                    setPathState(PathState.WIGGLE_FORWARD);
                }
                break;
            case WIGGLE_FORWARD:
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.INTAKE);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.IN);

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(driveToShoot);
                    setPathState(PathState.DRIVE_PICKUP_TO_SHOOT);
                }
                break;
            case DRIVE_PICKUP_TO_SHOOT:
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP);
                }
                break;
            case SHOOT_PICKUP:
                flywheelSubsystem.setVelocity(FlywheelConstants.FAR_SP);
                turretSubsystem.setPosition(TurretConstants.RED_FAR_SP);

                feederSubsystem.setStopperState(FeederConstants.STOPPER_STATE.OPEN);

                if (flywheelSubsystem.atVelocity() && !hasSpunUp) {
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
                    flywheelSubsystem.stop();

                    follower.followPath(driveToOffline);
                    setPathState(PathState.OFFLINE);
                }
                break;
            case OFFLINE:
                if (!follower.isBusy()) {
                    pathState = PathState.END;
                }
                break;
            case END:
                turretSubsystem.setPosition(0);
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
    }

    private void buildPaths() {
        driveToPickup = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        driveToOffline = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, offlinePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        wiggleBack = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, wigglePose))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();

        wiggleForward = follower.pathBuilder()
                .addPath(new BezierLine(wigglePose, pickupPose))
                .setConstantHeadingInterpolation(pickupPose.getHeading())
                .build();
    }
}
