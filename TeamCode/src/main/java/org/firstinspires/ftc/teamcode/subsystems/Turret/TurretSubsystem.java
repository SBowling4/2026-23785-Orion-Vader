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

import org.firstinspires.ftc.lib.orion.util.PoseConverter;
import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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
    private double manualAdjust = 0.0;


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

        pidController = new PIDFController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD,
                TurretConstants.kF
        );

    }

    public void loop() {
        if (Robot.tuningMode) {
            setPosition(TurretConstants.target);
        } else {
            if (Robot.mode == Robot.RobotMode.VADER) {
                turretSetpoint = findPosition();

                if (gamepad2.dpad_left) {
                    manualAdjust += .1;
                } else if (gamepad2.dpad_right) {
                    manualAdjust -= .1;
                } else if (gamepad2.x) {
                    manualAdjust = 0;
                }

                if (gamepad1.right_bumper) {
                    setPosition(turretSetpoint + manualAdjust);

                } else {
                    stop();
                }
            } else if (Robot.mode == Robot.RobotMode.KAOS) {
                if (gamepad2.dpad_left) {
                    setTurretPower(-.5);
                } else if (gamepad2.dpad_right) {
                    setTurretPower(.5);
                } else {
                    setTurretPower(0);
                }
            }

        }
    }

    public void setPosition(double pos) {
        pos = Range.clip(pos, -Math.PI/2, Math.PI/2);

        turretPower = -pidController.calculate(getPosition(), pos);
        setTurretPower(turretPower);
    }


    public Pose2d getTurretFieldPose() {
        // Get the FTC pose instead
        Pose2D ftcRobotPose = driveSubsystem.getEstimatedPoseFTC();

        // Turret is 3.376 inches behind the robot center
        double offsetMeters = Units.inchesToMeters(3.376);

        // Get robot's heading in FTC coordinates
        double robotHeading = ftcRobotPose.getHeading(AngleUnit.RADIANS);

        // In FTC robot coordinates, "back" is -Y direction
        // So we calculate the offset in field coordinates
        double turretOffsetX = -offsetMeters * Math.cos(robotHeading); // FTC: X is perpendicular to heading
        double turretOffsetY = -offsetMeters * Math.sin(robotHeading); // FTC: Y is along heading

        // Add the offset to the robot's position
        double turretX = ftcRobotPose.getX(DistanceUnit.METER) + turretOffsetX;
        double turretY = ftcRobotPose.getY(DistanceUnit.METER) + turretOffsetY;

        // Convert back to WPILib Pose2d
        Pose3D ftcTurretPose = new Pose3D(
                new Position(DistanceUnit.METER, turretX, turretY, 0.0, 0),
                new YawPitchRollAngles(AngleUnit.RADIANS, robotHeading, 0, 0, 0)
        );

        return PoseConverter.ftcToWPILib(ftcTurretPose).toPose2d();
    }

    public double findPosition() {
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

        robotHeading = driveSubsystem.getEstimatedPoseFTC().getHeading(AngleUnit.RADIANS);

        if (robotHeading < 0) {
            robotHeading += (2 * Math.PI);
        }

        double target = overallAngle - robotHeading;

        if (target < -Math.PI / 2.0 || target > Math.PI / 2.0) {
            return 0;
        }

        return target + .1;//0.77; //0.175
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

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Turret/Position", Units.radiansToDegrees(getPosition()));
        packet.put("Turret/Setpoint", Units.radiansToDegrees(turretSetpoint));
        packet.put("Turret/Needed Angle)", Units.radiansToDegrees(findPosition()));
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
