package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

public class DriveSubsystem {

    private MotorEx frontLeft, frontRight, backLeft, backRight;

    private PIDController alignPID;
    public MecanumDrive mecanum;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    public Follower follower;

    private Pose pose;

    private double lastHeading = 0;

    private Vision vision;

    private static DriveSubsystem instance;




    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
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

        alignPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        vision = Vision.getInstance();


//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose());
    }

//    public void start() {
//        follower.startTeleopDrive();
//    }

    public void loop(){
        alignPID.setP(DriveConstants.kP);
        alignPID.setD(DriveConstants.kD);

        mecanum.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

//        follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                false
//        );


        if (gamepad1.share) {
            resetHeading();
        }

        setTelemetry();
        addVisionMeasurement(vision.getVisPose());

//        follower.update();

    }

    public Pose getPose() {
        return new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading() + lastHeading, PedroCoordinates.INSTANCE);
    }

    public void resetHeading() {
        lastHeading = follower.getHeading();

        follower.setPose(follower.getPose().setHeading(0));

    }

    public void addVisionMeasurement(Pose3D visPose) {
        Pose pose = new Pose(visPose.getPosition().x, visPose.getPosition().y, visPose.getOrientation().getYaw(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

//        KalmanFilter filter = new KalmanFilter(new KalmanFilterParameters(0 ,0)); //TODO: Figure how this works, and if I should use it
//
//        filter.update();

        follower.setPose(pose);


    }

    public double getDistanceToGoal() {
        if (Robot.alliance == Alliance.BLUE) {
            double x = pose.getX();
            double y = 144 - pose.getY();

            return Math.hypot(x, y);
        } else if (Robot.alliance == Alliance.RED) {
            double x = 144 - pose.getX();
            double y = 144 - pose.getY();

            return Math.hypot(x, y);
        } else {
            throw new IllegalStateException("Alliance not set");
        }
    }




    public void stop() {
        frontLeft.stopMotor();
        backLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
    }

    private void setTelemetry() {
        telemetry.addLine("//Drive//");
        telemetry.addData("X", getPose().getX());
        telemetry.addData("Y", getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(getPose().getHeading()));
        telemetry.addData("Distance to Goal", getDistanceToGoal());
    }

    public static DriveSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new DriveSubsystem(hardwareMap, gamepad1, telemetry);
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