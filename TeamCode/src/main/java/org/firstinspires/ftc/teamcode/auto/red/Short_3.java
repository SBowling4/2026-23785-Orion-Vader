package org.firstinspires.ftc.teamcode.auto.red;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
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

@Autonomous(name="Short_3_Red")
public class Short_3 extends OpMode {
    private Follower follower;
    private Timer autoTimer, pathTimer;

    ShooterSubsystem shooterSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    IntakeSubsystem intakeSubsystem;
    Vision vision;

    public enum PathState {
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_READY_PICKUP,
        END
    }

    private PathState pathState;

    private final Pose startPose = new Pose(128.4043321299639, 127.53790613718411, Math.toRadians(37));
    private final Pose shootPose = new Pose(93.74729241877256, 93.74729241877256, Math.toRadians(48));
    private final Pose readyPickupPose = new Pose(106.74368231046931, 83.69675090252709, 0);


    private PathChain driveStartShoot, driveReadyPickup;

    public void buildPaths() {
        driveStartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveReadyPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, readyPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), readyPickupPose.getHeading())
                .build();
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartShoot, false);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooterSubsystem.shoot(false);
                    feederSubsystem.autoFeed();
                }

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 8) {
                    follower.followPath(driveReadyPickup);
                    setPathState(PathState.DRIVE_SHOOT_READY_PICKUP);
                }
                break;
            case DRIVE_SHOOT_READY_PICKUP:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
            case END:
                flywheelSubsystem.stop();
                shooterSubsystem.setAngle(0);
                feederSubsystem.stop();
            default:
                break;
        }
    }


    @Override
    public void init() {
        Robot.alliance = Alliance.RED;

        Robot.sendHardwareMap(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;

        pathTimer = new Timer();
        autoTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);



        intakeSubsystem.init();
        flywheelSubsystem.init();
        shooterSubsystem.init();
        feederSubsystem.init();
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
