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

@Autonomous(name = "Short_9_Red")
public class Short_9 extends OpMode {
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
        DRIVE_READY_FIRST_PICKUP_POS,
        FIRST_PICKUP,
        DRIVE_BACK_FIRST_SHOOT_POS,
        SHOOT_FIRST_PICKUP,
        DRIVE_READY_SECOND_PICKUP,
        SECOND_PICKUP,
        DRIVE_BACK_SECOND_PICKUP,
        SHOOT_SECOND_PICKUP,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(17.67509025270758, 121.64620938628158, Math.toRadians(144)).mirror();
    private final Pose shootPose = new Pose(45.92057761732852, 102.23826714801444, Math.toRadians(144)).mirror();
    private final Pose firstPickupCP = new Pose(57.53068592057762, 86.29602888086643, Math.toRadians(180)).mirror();
    private final Pose readyFirstPickupPose = new Pose(44.88086642599278, 83.35018050541515, Math.toRadians(180)).mirror();
    private final Pose firstPickupPose = new Pose(14.382671480144404, 83.87003610108303, Math.toRadians(180)).mirror();
    private final Pose readySecondPickupPose = new Pose(41.761732851985556, 59.43682310469315, Math.toRadians(180)).mirror();
    private final Pose secondPickupCP = new Pose(57.357400722021666, 69.48736462093864, Math.toRadians(180)).mirror();
    private final Pose secondPickupPose = new Pose(22.0072202166065, 60.64981949458484, Math.toRadians(180)).mirror();
    private final Pose offlinePose = new Pose(28.76534296028881, 77.11191335740072, Math.toRadians(180)).mirror();

    private PathChain driveStartShoot, driveReadyFirstPickup, driveFirstPickup, driveFirstPickupShoot, driveReadySecondPickup, driveSecondPickup, driveSecondPickupShoot, driveOffline;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveReadyFirstPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, firstPickupCP, readyFirstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyFirstPickupPose.getHeading())
                .build();

        driveFirstPickup = follower.pathBuilder()
                .addPath(new BezierLine(readyFirstPickupPose, firstPickupPose))
                .setLinearHeadingInterpolation(readyFirstPickupPose.getHeading(), firstPickupPose.getHeading())
                .build();

        driveFirstPickupShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();

        driveReadySecondPickup = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, secondPickupCP, readySecondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readySecondPickupPose.getHeading())
                .build();

        driveSecondPickup = follower.pathBuilder()
                .addPath(new BezierLine(readySecondPickupPose, secondPickupPose))
                .setLinearHeadingInterpolation(readyFirstPickupPose.getHeading(), secondPickupPose.getHeading())
                .build();

        driveSecondPickupShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, secondPickupCP, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
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

                    follower.followPath(driveReadyFirstPickup);
                    setPathState(PathState.DRIVE_READY_FIRST_PICKUP_POS);
                }
                break;
            case DRIVE_READY_FIRST_PICKUP_POS:
                if (!follower.isBusy()) {
                    follower.followPath(driveFirstPickup, .5, true);
                    setPathState(PathState.FIRST_PICKUP);
                }
                break;
            case FIRST_PICKUP:
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
                    follower.followPath(driveFirstPickupShoot);
                    setPathState(PathState.DRIVE_BACK_FIRST_SHOOT_POS);
                }
                break;
            case DRIVE_BACK_FIRST_SHOOT_POS:
                intakeSubsystem.stop();
                feederSubsystem.stop();
                flywheelSubsystem.stop();



                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_FIRST_PICKUP);
                }
                break;

            case SHOOT_FIRST_PICKUP:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                    intakeSubsystem.intake();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveReadySecondPickup);
                    setPathState(PathState.DRIVE_READY_SECOND_PICKUP);
                }
                break;
            case DRIVE_READY_SECOND_PICKUP:
                flywheelSubsystem.stop();
                intakeSubsystem.stop();
                feederSubsystem.stop();

                if (!follower.isBusy()) {
                    follower.followPath(driveSecondPickup, .5, true);
                    setPathState(PathState.SECOND_PICKUP);
                }
                break;
            case SECOND_PICKUP:
                if (pathTimer.getElapsedTimeSeconds() < 1.67) {
                    intakeSubsystem.intake();
                    feederSubsystem.feed();
                    flywheelSubsystem.setPower(1);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.67 && pathTimer.getElapsedTimeSeconds() < 2) {
                    intakeSubsystem.stop();
                    feederSubsystem.stop();
                    flywheelSubsystem.stop();
                }


//                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() < 2.5) {
//                    intakeSubsystem.intake();
//                    flywheelSubsystem.setPower(1);
//                    feederSubsystem.back();
//                }

//                if (!follower.isBusy() & pathTimer.getElapsedTimeSeconds() > 2.5 && pathTimer.getElapsedTimeSeconds() < 2.75) {
//                    intakeSubsystem.intake();
//                    flywheelSubsystem.setPower(1);
//                    feederSubsystem.feed();
//                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    feederSubsystem.stop();
                    intakeSubsystem.stop();
                    flywheelSubsystem.stop();

                    follower.followPath(driveSecondPickupShoot);
                    setPathState(PathState.DRIVE_BACK_SECOND_PICKUP);
                }
                break;
            case DRIVE_BACK_SECOND_PICKUP:
                feederSubsystem.stop();
                intakeSubsystem.stop();
                flywheelSubsystem.stop();

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SECOND_PICKUP);
                }
                break;
            case SHOOT_SECOND_PICKUP:
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
                intakeSubsystem.stop();
                feederSubsystem.stop();
                flywheelSubsystem.stop();

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

        telemetry.addLine("//Odometry//");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
    }

}
