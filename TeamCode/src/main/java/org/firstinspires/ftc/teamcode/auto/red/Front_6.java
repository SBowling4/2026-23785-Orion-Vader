package org.firstinspires.ftc.teamcode.auto.red;

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
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

@Autonomous(name = "Front_6_Red")
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

    private Paths paths;

    private Pose startPose = new Pose(18, 121.54, Math.toRadians(144));
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
        paths = new Paths(follower);

        pathState = PathState.DRIVE_SHOOT_POS;

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);
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

        driveSubsystem.autoLoop(follower);

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
                follower.followPath(paths.driveStartToShoot);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity(driveSubsystem.getDistanceToGoal()));
                turretSubsystem.setPosition(turretSubsystem.findPosition());

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

                    follower.followPath(paths.drivePickup);
                    setPathState(PathState.PICKUP);
                }
                break;
            case PICKUP:
                if (!follower.isBusy()) {
                    follower.followPath(paths.drivePickupToShoot);
                    setPathState(PathState.DRIVE_PICKUP_SHOOT);
                }
                break;
            case DRIVE_PICKUP_SHOOT:
                intakeSubsystem.setState(IntakeConstants.INTAKE_STATE.STOP);
                feederSubsystem.setFeederState(FeederConstants.FEEDER_STATE.STOP);

                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity(driveSubsystem.getDistanceToGoal()));
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PICKUP);
                }
                break;
            case SHOOT_PICKUP:
                flywheelSubsystem.setVelocity(flywheelSubsystem.findVelocity(driveSubsystem.getDistanceToGoal()));
                turretSubsystem.setPosition(turretSubsystem.findPosition());

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

                    follower.followPath(paths.driveOffline);
                    setPathState(PathState.OFFLINE);
                }
                break;
            case OFFLINE:
                if (!follower.isBusy()) {
                    setPathState(PathState.END);
                }
                break;
            case END:
                terminateOpModeNow();

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



    public static class Paths {
        public PathChain driveStartToShoot;
        public PathChain drivePickup;
        public PathChain drivePickupToShoot;
        public PathChain driveOffline;

        public Paths(Follower follower) {
            driveStartToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.000, 121.540),

                                    new Pose(59.186, 84.903)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                    .build();

            drivePickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.186, 84.903),

                                    new Pose(14.929, 84.062)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            drivePickupToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.929, 84.062),

                                    new Pose(58.788, 85.389)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            driveOffline = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(58.788, 85.389),

                                    new Pose(20.124, 70.221)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }


}
