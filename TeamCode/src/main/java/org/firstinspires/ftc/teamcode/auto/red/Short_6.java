package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "Short_6_Red")
public class Short_6 extends OpMode {
    private Follower follower;
    private Timer autoTimer, pathTimer;

    ShooterSubsystem shooterSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;
    Vision vision;

    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_READY_PICKUP_POS,
        PICKUP,
        DRIVE_BACK_SHOOT_POS,
        SHOOT_PICKUP,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(17.67509025270758, 121.64620938628158, Math.toRadians(144)).mirror();
    private final Pose shootPose = new Pose(45.92057761732852, 102.23826714801444, Math.toRadians(144)).mirror();
    private final Pose pickupCP = new Pose(57.53068592057762, 86.29602888086643, Math.toRadians(180)).mirror();
    private final Pose readyPickupPose = new Pose(44.88086642599278, 83.35018050541515, Math.toRadians(180)).mirror();
    private final Pose pickupPose = new Pose(14.382671480144404, 83.87003610108303, Math.toRadians(180)).mirror();
    private final Pose offlinePose = new Pose(28.76534296028881, 77.11191335740072, Math.toRadians(180)).mirror();

    private PathChain driveStartShoot, driveReadyPickup, drivePickup, drivePickupShoot, driveOffline;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveReadyPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupCP, readyPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyPickupPose.getHeading())
                .build();

        drivePickup = follower.pathBuilder()
                .addPath(new BezierLine(readyPickupPose, pickupPose))
                .setLinearHeadingInterpolation(readyPickupPose.getHeading(), pickupPose.getHeading())
                .build();

        drivePickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();

        driveOffline = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, offlinePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), offlinePose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                    intakeSubsystem.intake();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    flywheelSubsystem.stop();
                    feederSubsystem.stop();
                    intakeSubsystem.stop();

                    follower.followPath(driveReadyPickup);
                    setPathState(PathState.DRIVE_READY_PICKUP_POS);
                }
                break;
            case DRIVE_READY_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.followPath(drivePickup, .5, true);
                    setPathState(PathState.PICKUP);
                }
                break;
            case PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 1.75) {
                    intakeSubsystem.intake();
                    feederSubsystem.feed();
                    flywheelSubsystem.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.75 && pathTimer.getElapsedTimeSeconds() < 2) {
                    intakeSubsystem.stop();
                    feederSubsystem.stop();
                    flywheelSubsystem.stop();
                }


                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() < 2.5) {
                    intakeSubsystem.intake();
                    flywheelSubsystem.setPower(1);
                    feederSubsystem.back();
                }

                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.75) {
                    intakeSubsystem.intake();
                    flywheelSubsystem.setPower(1);
                    feederSubsystem.feed();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(drivePickupShoot);
                    setPathState(PathState.DRIVE_BACK_SHOOT_POS);
                }
                break;
            case DRIVE_BACK_SHOOT_POS:
                intakeSubsystem.stop();
                feederSubsystem.stop();
                flywheelSubsystem.stop();



                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP);
                }
                break;

            case SHOOT_PICKUP:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                    intakeSubsystem.intake();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveOffline);
                    setPathState(PathState.DRIVE_OFFLINE);
                }
                break;
            case DRIVE_OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
            case END:
                terminateOpModeNow();
                break;
            default:
                break;
        }
    }


    @Override
    public void init() {
        Robot.alliance = Alliance.RED;
        Robot.sendHardwareMap(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        pathState = PathState.DRIVE_START_POS_SHOOT_POS;

        pathTimer = new Timer();
        autoTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);


        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);

        flywheelSubsystem.init();
        shooterSubsystem.init();
        feederSubsystem.init();
        intakeSubsystem.init();
        vision.init();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        vision.start();

        autoTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        vision.loop();

        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState.toString());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addLine();


        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
        telemetry.addLine();

        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
        telemetry.addLine();



        telemetry.addLine("//Vision//");
        telemetry.addData("LL Valid", vision.llValid);
        telemetry.addData("Has Tag", vision.hasTag);
        telemetry.addData("Ta", vision.getTa().orElse(-1.0));
        telemetry.addData("Tx", vision.getTx().orElse(-1.0));
        telemetry.addData("Ty", vision.getTy().orElse(-1.0));
        telemetry.addData("Distance", vision.getDistance().orElse(-1.0));
        telemetry.addLine();
    }

}
