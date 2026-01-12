package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.PoseConverter;
import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Transform2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

    private double turretSetpoint = 0.0;
    private double turretPower = 0.0;

    double overall;
    double heading;

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
        encoder = new MotorEx(hardwareMap, DriveConstants.LEFT_BACK_MOTOR_NAME).encoder;

        pidController = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );

    }

    public void loop() {
//        if (Robot.tuningMode) {
//            setPosition(TurretConstants.target);
//        } else {
            turretSetpoint = findPosition();

            setPosition(turretSetpoint);
//        }
    }

    public void setPosition(double pos) {
        turretPower = -pidController.calculate(getPosition(), pos);
        setTurretPower(turretPower);
    }

    public Pose2d getTurretFieldPose() {
        Pose2d robotPose = driveSubsystem.getEstimatedPoseWPILIB();

        // Convert offset from inches to meters
        double offsetMeters = Units.inchesToMeters(-3.376); // Negative because it's back from center

        // Create a transform representing only the turret's physical offset from robot center
        // No rotation component - we only care about WHERE the turret is, not its angle
        Transform2d turretOffset = new Transform2d(
                new Translation2d(offsetMeters, 0.0), // X offset (backward), no Y offset
                new Rotation2d(0.0) // No rotation in the transform
        );

        // Apply the transform to get the turret's field position
        Pose2d turretPose = robotPose.transformBy(turretOffset);

        return turretPose;
    }

    public double findPosition() {
        double robotHeading;
        double overallAngle;

        Pose2d turretPose = getTurretFieldPose();

        if (Robot.alliance == Alliance.BLUE) {
            Translation2d delta = turretPose.getTranslation().minus(Field.BLUE_GOAL);

            overallAngle = Math.PI + delta.getAngle().getRadians();

        } else if (Robot.alliance == Alliance.RED) {
            Translation2d delta = turretPose.getTranslation().minus(Field.RED_GOAL);

            overallAngle = delta.getAngle().getRadians();
        } else {
            throw new IllegalStateException("Alliance not set");
        }

        robotHeading = driveSubsystem.getEstimatedPoseFTC().getHeading(AngleUnit.RADIANS);

        if (robotHeading < 0) {
            robotHeading += (2 * Math.PI);
        }

        double target = overallAngle - robotHeading;

        if (target < -Math.PI / 2.0 || target > Math.PI / 2.0) {
            return 0;
        }

        return target;
    }

    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) encoder.getPosition() / ticksPerRev;

        return -(-revolutions * 2 * Math.PI * TurretConstants.GEAR_RATIO);
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
        telemetry.addData("Turret Angle (rad)", getPosition());
        telemetry.addData("Turret Setpoint (rad)", turretSetpoint);
        telemetry.addData("Turret Needed Angle", findPosition());
        telemetry.addData("Turret Overall Needed", overall);
        telemetry.addData("Heading", heading);
        telemetry.addLine();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Turret/Position", Units.radiansToDegrees(getPosition()));
        packet.put("Turret/Setpoint", Units.radiansToDegrees(turretSetpoint));
        packet.put("Turret/Needed Angle)", Units.radiansToDegrees(findPosition()));
        packet.put("Turret/Pose/Pose x", Units.metersToInches(getTurretFieldPose().getTranslation().getX()));
        packet.put("Turret/Pose/Pose y", Units.metersToInches(getTurretFieldPose().getTranslation().getY()));
        packet.put("Turret/Pose/Pose heading", getTurretFieldPose().getRotation().getRadians());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
