package org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class HoodSubsystem {
    public CRServoImplEx hoodServo;
    public Motor.Encoder hoodEncoder;
    public PIDController pid;
    private FlywheelSubsystem flywheelSubsystem;
    private Vision vision;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;
    private static HoodSubsystem instance;

    public double tuningPos = 0;
    public double targetPos = 0;
    private boolean last = false;

    /**
     * Shooter Subsystem constructor
     */
    public HoodSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the Shooter Subsystem
     */
    public void init() {
        hoodServo = hardwareMap.get(CRServoImplEx.class, HoodConstants.SERVO_NAME);

        hoodEncoder = FlywheelSubsystem.getInstance().rightMotor.encoder;

        pid = new PIDController(
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD
        );

        double ticksPerRev = 8192;
        double degreesPerPulse = (360.0 * HoodConstants.GEAR_RATIO) / ticksPerRev;

        hoodEncoder.setDistancePerPulse(degreesPerPulse);

        hoodEncoder.reset();

        pid.setTolerance(1);

        flywheelSubsystem = FlywheelSubsystem.getInstance();
        vision = Vision.getInstance();

        tuningPos = 0;
    }

    /**
     * Loops the Shooter Subsystem
     */
    public void loop() {
//        if (Robot.tuningMode) {
//            if (gamepad1.dpad_up) {
//                tuningPos += .5;
//            } else if (gamepad1.dpad_down) {
//                tuningPos -= .5;
//            }
//
//            tuningPos = Range.clip(tuningPos, 0, 25);
//
//            setAngle(tuningPos);
//        }





        setTelemetry();

    }

    /**
     *
     * @return if the shooter is at the target position
     */
    public boolean atPosition() {
        return Math.abs(getPosition() - targetPos) < 1;
    }

    /**
     * Shoots the Artifact using the limelight. Calculates the angle and velocity using the distance from the tag from the Limelight
     *
     * @param isBack If the default shooter mode (if no tag seen) should be true if far, false if short
     */
    public void shoot(boolean isBack) {
//        if (vision.getDistance().isEmpty() && isBack) {
//            setAngle(ShooterConstants.FAR_ANGLE);
//            flywheelSubsystem.setVelocity(FlywheelConstants.FAR_AUTO_VELOCITY);
//
//            return;
//        }
//
//        if (vision.getDistance().isEmpty() && !isBack) {
//            setAngle(ShooterConstants.CLOSE_ANGLE);
//            flywheelSubsystem.setVelocity(FlywheelConstants.CLOSE_VELOCITY);
//
//            return;
//        }
//
//
//        if (vision.getDistance().isEmpty()) return;
//
//        double velocityFromDistance = flywheelSubsystem.findVelocity(vision.getDistance().get());
//        double angleFromDistance = findAngle(vision.getDistance().get());
//
//        setAngle(angleFromDistance);
//        flywheelSubsystem.setVelocity(velocityFromDistance);
    }



    /**
     *
     * @return the position of the shooter (degrees)
     */
    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) hoodEncoder.getPosition() / ticksPerRev;

        return revolutions * 360.0 * HoodConstants.GEAR_RATIO;
    }


    /**
     *
     * Sets the shooter to a specific angle
     *
     * @param targetAngle the angle for the shooter to go to (degrees)
     */
    public void setAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

        this.targetPos = targetAngle;

        double power = pid.calculate(getPosition(), targetAngle);
        hoodServo.setPower(power);
    }

    /**
     *
     * Equation obtained from here: <a href="https://docs.google.com/spreadsheets/d/1m6Tb_BewsEm0vuEWVIr-rKV5Jfy468Ui95xVuQbh-_I/edit?usp=sharing">Spreadsheet</a>
     *
     * @param distance distance (m) from target (Front of robot to base of goal)
     * @return Desired angle for shooter (degrees)
     */
    public double findAngle(double distance) {
        if (distance >= 1.425) return 25;
        return -151 + 427 * distance + -356 * Math.pow(distance, 2) + 101 * Math.pow(distance, 3);
    }

    private void setTelemetry() {
        telemetry.addLine("//Hood//");
        telemetry.addData("Position", getPosition());
        telemetry.addData("Target", targetPos);
        telemetry.addLine();
    }


    /**
     * Singleton pattern to get the instance of the ShooterSubsystem
     *
     * @param hardwareMap HardwareMap from the OpMode
     * @param gamepad1    Gamepad1 from the OpMode
     * @param telemetry    Telemetry from the OpMode
     * @return Instance of the ShooterSubsystem
     */
    public static HoodSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new HoodSubsystem(hardwareMap, gamepad1, telemetry);
        }
        return instance;
    }

    /**
     * Singleton pattern to get the instance of the ShooterSubsystem
     *
     * @return Instance of the ShooterSubsystem
     */
    public static HoodSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ShooterSubsystem not initialized. Call getInstance(hardwareMap, gamepad2) first.");
        }
        return instance;
    }
}