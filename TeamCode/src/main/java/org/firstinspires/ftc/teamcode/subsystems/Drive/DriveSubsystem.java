package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.Field;
import org.firstinspires.ftc.lib.orion.odometry.Odometry;
import org.firstinspires.ftc.lib.orion.odometry.PoseEstimator;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.lib.orion.util.Alliance;

public class DriveSubsystem {

    public MotorEx frontLeft, frontRight, backLeft, backRight;

    public MecanumDrive mecanum;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public Follower follower;

    public Odometry odometry;
    public PoseEstimator poseEstimator;

    private Pose pose;

    private double lastHeading = 0;

    private static DriveSubsystem instance;




    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        frontLeft = new MotorEx(hardwareMap, DriveConstants.LEFT_FRONT_MOTOR_NAME);
        frontRight = new MotorEx(hardwareMap, DriveConstants.RIGHT_FRONT_MOTOR_NAME);
        backLeft = new MotorEx(hardwareMap, DriveConstants.LEFT_BACK_MOTOR_NAME);
        backRight = new MotorEx(hardwareMap, DriveConstants.RIGHT_BACK_MOTOR_NAME);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        frontRight.setInverted(true);
        backRight.setInverted(true);
        frontLeft.setInverted(true);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        odometry = new Odometry();

        poseEstimator = new PoseEstimator(odometry, VecBuilder.fill(0,0,0), VecBuilder.fill(0,0,0)); //TODO: fill these in



//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose()); //TODO: idk smthn here
    }

    public void start() {
//        follower.startTeleopDrive();
    }

    public void loop() {

        mecanum.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                false
//        );


//        if (gamepad1.share) {
//            resetHeading();
//        }

//        follower.update();
//        odometry.update(getFollowerPose());
//        poseEstimator.update();

    }

    private Pose getFollowerPose() {
        return new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading() + lastHeading);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetHeading() {
        lastHeading += follower.getHeading();

        follower.setPose(follower.getPose().setHeading(0));

    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds);
    }

    public double getDistanceToGoal() {
        if (Robot.alliance == Alliance.BLUE) {
            return getEstimatedPose().getTranslation().getDistance(Field.BLUE_GOAL);
        } else {
            return getEstimatedPose().getTranslation().getDistance(Field.RED_GOAL);
        }
    }




    public void stop() {
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Drive//");
        telemetry.addData("X", getEstimatedPose().getX());
        telemetry.addData("Y", getEstimatedPose().getY());
        telemetry.addData("Heading", getEstimatedPose().getRotation().getDegrees());
        telemetry.addData("Distance to Goal", getDistanceToGoal());
    }

    public static DriveSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new DriveSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("DriveSubsystem not initialized. Call getInstance(telemetry, hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}