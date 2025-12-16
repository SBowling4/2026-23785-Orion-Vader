package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
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
    private DriveSubsystem driveSubsystem;

    private static TurretSubsystem instance;


    /**
     * Private constructor for singleton pattern.
     *
     * @param hardwareMap Hardware map from the robot.
     */
    private TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
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

        driveSubsystem = DriveSubsystem.getInstance();

    }

    //TODO: Tune PID
    //TODO: Figure out how absolute encoders work
    /**
     * Main loop for turret subsystem.
     */
    public void loop() {
        robotPose = driveSubsystem.getPose();

        if (Robot.alliance == Alliance.BLUE) {
            turretAngle = getPosition();

            double x = robotPose.getX();
            double y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = 180 - Math.atan2(y, x);

            turretSetpoint = overallAngle - robotHeading;

            double pidOutput = pidController.calculate(turretAngle, turretSetpoint);

            turretServo.setPower(pidOutput);
        } else if (Robot.alliance == Alliance.RED) {
            turretAngle = getPosition();

            double x = 144 - robotPose.getX();
            double y = 144 - robotPose.getY();

            robotHeading = robotPose.getHeading();
            overallAngle = Math.atan2(y, x);

            turretSetpoint = robotHeading - overallAngle;

            double pidOutput = pidController.calculate(turretAngle, turretSetpoint);

            turretServo.setPower(pidOutput);

        } else {
            throw new IllegalStateException("Alliance not set");
        }


    }

    /**
     * Get turret position in radians.
     *
     * @return Turret position in radians.
     */
    public double getPosition() {
        double voltage = encoder.getVoltage() - TurretConstants.OFFSET;

        return (voltage / 3.3) * 2 * Math.PI * TurretConstants.GEAR_RATIO;
    }

    /**
     * Stop turret movement (hold current angle).
     */
    public void stop() {
        turretPower = 0;
        turretServo.setPower(0.0);
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap);
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
