package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.lib.orion.util.Alliance;

@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem {

    private CRServoImplEx turretServo;
    private Motor.Encoder encoder;
    private PIDFController pidController;

    private Pose robotPose;

    private double setpoint = 0.0;
    private double power = 0.0;
    private double position = 0.0;


    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private DriveSubsystem driveSubsystem;
    private static TurretSubsystem instance;


    private TurretSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void init() {
         driveSubsystem = DriveSubsystem.getInstance();

        turretServo = hardwareMap.get(CRServoImplEx.class, TurretConstants.TURRET_SERVO_NAME);
        encoder = new MotorEx(hardwareMap, DriveConstants.LEFT_BACK_MOTOR_NAME).encoder;

        encoder.reset();

        pidController = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );

    }

    public void loop() {
        position = getPosition();
        setpoint = findFieldRelativeAngle();

//        setPosition(setpoint);
    }

    public void setPosition(double pos) {
        pos = Range.clip(pos, -Math.PI/2, Math.PI/2);

        power = -pidController.calculate(position, pos);
        setPower(power);
    }


    public Pose2d getTurretFieldPose() {
        Pose2d robotPose = driveSubsystem.getEstimatedPose();

        Transform2d robotToTurret = new Transform2d(Units.inchesToMeters(-3.376), 0, Rotation2d.kZero);

        return robotPose.transformBy(robotToTurret);
    }

    public double findFieldRelativeAngle() {
        double robotHeading;
        double overallAngle;

        Translation2d turretPose = getTurretFieldPose().getTranslation();

        if (Robot.alliance == Alliance.BLUE) {
            Translation2d delta = turretPose.minus(Field.BLUE_GOAL);

            overallAngle = Math.PI + delta.getAngle().getRadians();

        } else if (Robot.alliance == Alliance.RED) {
            Translation2d delta = turretPose.minus(Field.RED_GOAL);

            overallAngle = delta.getAngle().getRadians();
        } else {
            throw new IllegalStateException("Alliance not set");
        }

        robotHeading = driveSubsystem.getEstimatedPose().getRotation().getRadians();

        double target = overallAngle - robotHeading;

        if (target < -Math.PI / 2.0 || target > Math.PI / 2.0) {
            return 0;
        }

        return target;
    }

    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) position / ticksPerRev;

        return -revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO;
    }


    public void stop() {
        power = 0.0;
        turretServo.setPower(0.0);
    }

    public void setPower(double power) {
        this.power = power;
        turretServo.setPower(power);
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Turret/Position", Units.radiansToDegrees(getPosition()));
        packet.put("Turret/Setpoint", Units.radiansToDegrees(setpoint));
        packet.put("Turret/Needed Angle)", Units.radiansToDegrees(findFieldRelativeAngle()));
        packet.put("Turret/Pose/Pose x", Units.metersToInches(getTurretFieldPose().getX()));
        packet.put("Turret/Pose/Pose y", Units.metersToInches(getTurretFieldPose().getY()));
        packet.put("Turret/Pose/Pose heading", 0);
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap, gamepad1, gamepad2);
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
