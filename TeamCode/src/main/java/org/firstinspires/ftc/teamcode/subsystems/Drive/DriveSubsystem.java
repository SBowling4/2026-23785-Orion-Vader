package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.lib.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class DriveSubsystem {

    private MotorEx frontLeft, frontRight, backLeft, backRight;

    private IMU imu;

    private PIDController alignPID;
    public MecanumDrive mecanum;

    public final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Vision vision;

    private static DriveSubsystem instance;

//    public Follower follower;



    public DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
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

        alignPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));



        vision = Vision.getInstance(hardwareMap);

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose());
    }

//    public void start() {
//        follower.startTeleopDrive();
//    }

    public void loop(){
        alignPID.setP(DriveConstants.kP);
        alignPID.setD(DriveConstants.kD);

        mecanum.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw());

//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                false
//        );


        if (gamepad1.x) {
            align();
            return;
        }

        if (gamepad1.share) {
            resetHeading();
        }

//        follower.update();

    }


    public void align() {
        if (vision.getTx().isEmpty()) {
            mecanum.driveRobotCentric(0, 0, 0);
//            follower.setTeleOpDrive(0,0,0);
            return;
        }

        double tx = vision.getTx().get();

        double target = 0;

        double power = alignPID.calculate(tx, target);

        mecanum.driveRobotCentric(0, 0, power);

//        follower.setTeleOpDrive(0, 0, power);
//        follower.update();
    }

//    public Pose getPose() {
//        return follower.getPose();
//    }

    public void resetHeading() {
        imu.resetYaw();
//        follower.setPose(follower.getPose().setHeading(0));

    }




    public void stop() {
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
    }

    public static DriveSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new DriveSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public DriveSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("DriveSubsystem not initialized. Call getInstance(telemetry, hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}