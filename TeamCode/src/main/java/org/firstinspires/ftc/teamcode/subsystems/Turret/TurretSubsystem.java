package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem {

    private CRServoImplEx turretServo;
    private Motor.Encoder encoder;
    private PIDFController pidController;

    private Pose robotPose;

    private double turretAngle = Math.PI / 2.0;
    private double turretSetpoint = 0.0;
    private double turretPower = 0.0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private DriveSubsystem driveSubsystem;
    private static TurretSubsystem instance;


    private TurretSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
         driveSubsystem = DriveSubsystem.getInstance();

        turretServo = hardwareMap.get(CRServoImplEx.class, TurretConstants.TURRET_SERVO_NAME);
        encoder = driveSubsystem.backLeft.encoder;

        pidController = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );

    }

    public void loop() {
//        robotPose = driveSubsystem.getPose();

        // Always update measured position
        turretAngle = getPosition();

        if (gamepad1.right_bumper) {
            turretSetpoint = findPosition();
        } else {
            turretSetpoint = 0.0;
        }

        setPosition(turretSetpoint);
    }

    public void setPosition(double pos) {
        turretPower = -pidController.calculate(turretAngle, pos);
        setTurretPower(turretPower);
    }

    public double findPosition() {
        double x, y;
        double robotHeading;
        double overallAngle;

        if (Robot.alliance == Alliance.BLUE) {
            x = robotPose.getX();
            y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = Math.PI - Math.atan2(y, x);

        } else if (Robot.alliance == Alliance.RED) {
            x = 144 - robotPose.getX();
            y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = Math.atan2(y, x);

        } else {
            return 0.0;
        }

        double target = overallAngle - robotHeading;

        if (target < -Math.PI / 2.0 || target > Math.PI / 2.0) {
            return 0.0;
        }

        return target;
    }

    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) encoder.getPosition() / ticksPerRev;

        return -revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO;
    }


    public void stop() {
        turretPower = 0.0;
        turretServo.setPower(0.0);
    }

    public void setTurretPower(double power) {
        turretPower = power;
        turretServo.setPower(power);
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("// Turret //");
        telemetry.addData("Turret Angle (rad)", turretAngle);
        telemetry.addData("Turret Setpoint (rad)", turretSetpoint);
        telemetry.addData("Turret Error (deg)", (Math.abs(turretAngle - turretSetpoint) * 180) / Math.PI);
        telemetry.addLine();
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("TurretSubsystem not initialized.");
        }
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }
}
