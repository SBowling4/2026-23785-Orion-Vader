package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
    private AnalogInput encoder;
    private PIDController pidController;

    public double turretPower = 0.0;

    private Pose robotPose;
    private double turretAngle = 0.0;
    private double overallAngle = 0.0;
    private double robotHeading = 0.0;

    private double turretSetpoint = 0.0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;
    private DriveSubsystem driveSubsystem;

    private static TurretSubsystem instance;

    // =====================
    // Absolute encoder wrap handling
    // =====================
    private double lastRawAngle = 0.0;
    private double continuousAngle = 0.0;
    private boolean encoderInitialized = false;

    /**
     * Private constructor for singleton pattern.
     */
    private TurretSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    /**
     * Initialize turret subsystem.
     */
    public void init() {
        turretServo = hardwareMap.get(CRServoImplEx.class, TurretConstants.TURRET_SERVO_NAME);
        encoder = hardwareMap.get(AnalogInput.class, TurretConstants.TURRET_ENCODER_NAME);

        pidController = new PIDController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD
        );

//        driveSubsystem = DriveSubsystem.getInstance();
    }

    /**
     * Main loop for turret subsystem.
     */
    public void loop() {
        robotPose = driveSubsystem.getPose();

        // Always update turret angle (continuous, wrap-safe)
        turretAngle = getPosition();

        if (gamepad1.right_bumper) {
            turretSetpoint = findPosition();
        } else {
            turretSetpoint = 0;
        }

        turretPower = pidController.calculate(turretAngle, turretSetpoint);
        turretServo.setPower(turretPower);

        setTelemetry();
    }

    public double findPosition() {
        double x, y;

        if (Robot.alliance == Alliance.BLUE) {

            x = robotPose.getX();
            y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = Math.PI - Math.atan2(y, x);

            double target = overallAngle - robotHeading;

            if (target < -Math.PI || target > Math.PI) {
                return  0;
            }

            return target;

        } else if (Robot.alliance == Alliance.RED) {

            x = 144 - robotPose.getX();
            y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = Math.atan2(y, x);

            double target = overallAngle - robotHeading;

            if (target < -Math.PI || target > Math.PI) {
                return  0;
            }

            return target;

        }

        return -1;

    }

    /**
     * Get continuous turret position in radians (wrap-safe).
     */
    public double getPosition() {
        // 1. Absolute encoder angle (0 → 2π)
        double voltage = encoder.getVoltage();
        double encoderAngle =
                (voltage / encoder.getMaxVoltage()) * 2.0 * Math.PI;

        // Normalize encoder angle
        encoderAngle = (encoderAngle % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);

        // 2. Initialize unwrap
        if (!encoderInitialized) {
            lastRawAngle = encoderAngle;
            continuousAngle = 0.0; // encoder-space
            encoderInitialized = true;
        }

        // 3. Unwrap in ENCODER space
        double delta = encoderAngle - lastRawAngle;

        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        continuousAngle += delta;
        lastRawAngle = encoderAngle;

        // 4. Convert encoder rotation → turret rotation
        return continuousAngle / TurretConstants.GEAR_RATIO;
    }


    public double getRawPosition() {
        return encoder.getVoltage();
    }

    /**
     * Stop turret movement (hold current angle).
     */
    public void stop() {
        turretPower = 0;
        turretServo.setPower(0.0);
    }

    public void setTurretPower(double power) {
        turretPower = power;
        turretServo.setPower(power);
    }

    private void setTelemetry() {
        telemetry.addLine("//Turret//");
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Turret Setpoint", turretSetpoint);
        telemetry.addLine();
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap, gamepad1, telemetry);
        }
        return instance;
    }

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("TurretSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }
}
