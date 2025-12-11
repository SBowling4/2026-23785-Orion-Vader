package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class ShooterSubsystem {
    public CRServoImplEx servo;
    public Motor.Encoder encoder;
    private TouchSensor touchSensor;
    public PIDController pid;
    private FlywheelSubsystem flywheelSubsystem;
    private Vision vision;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private static ShooterSubsystem instance;

    public double tuningPos = 0;
    public double targetPos = 0;
    private boolean last = false;

    /**
     * Shooter Subsystem constructor
     */
    public ShooterSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        servo = hardwareMap.get(CRServoImplEx.class, ShooterConstants.SERVO_NAME);

        encoder = FlywheelSubsystem.getInstance().rightMotor.encoder;

        touchSensor = hardwareMap.get(TouchSensor.class, ShooterConstants.TOUCH_SENSOR_NAME);

        pid = new PIDController(
                ShooterConstants.kP,
                ShooterConstants.kI,
                ShooterConstants.kD
        );

        double ticksPerRev = 8192;
        double degreesPerPulse = (360.0 * ShooterConstants.GEAR_RATIO) / ticksPerRev;

        encoder.setDistancePerPulse(degreesPerPulse);

        encoder.reset();

        pid.setTolerance(1);

        flywheelSubsystem = FlywheelSubsystem.getInstance();
        vision = Vision.getInstance();

        tuningPos = 0;
    }

    public void loop() {
        if (getTS()) {
            encoder.reset();
        }

        if (Robot.tuningMode) {
            if (gamepad1.dpad_up) {
                tuningPos += .5;
            } else if (gamepad1.dpad_down) {
                tuningPos -= .5;
            }

            tuningPos = Range.clip(tuningPos, 0, 25);

            setAngle(tuningPos);
        }


        if (gamepad1.right_bumper) {
            shoot(false);
        }

    }

    public boolean atPosition() {
        return Math.abs(getPosition() - targetPos) < 1;
    }

    /**
     * Shoots the Artifact using the limelight. Calculates the angle and velocity using the distance from the tag from the Limelight
     *
     * @param isBack If the default shooter mode (if no tag seen) should be true if far, false if short
     */
    public void shoot(boolean isBack) {
        if (vision.getDistance().isEmpty() && isBack) {
            setAngle(ShooterConstants.FAR_ANGLE);
            flywheelSubsystem.setVelocity(FlywheelConstants.FAR_AUTO_VELOCITY);

            return;
        }

        if (vision.getDistance().isEmpty() && !isBack) {
            setAngle(ShooterConstants.CLOSE_ANGLE);
            flywheelSubsystem.setVelocity(FlywheelConstants.CLOSE_VELOCITY);

            return;
        }


        if (vision.getDistance().isEmpty()) return;

        double velocityFromDistance = flywheelSubsystem.findVelocity(vision.getDistance().get());
        double angleFromDistance = findAngle(vision.getDistance().get());

        setAngle(angleFromDistance);
        flywheelSubsystem.setVelocity(velocityFromDistance);
    }



    /**
     *
     * @return the position of the shooter (degrees)
     */
    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) encoder.getPosition() / ticksPerRev;

        return revolutions * 360.0 * ShooterConstants.GEAR_RATIO;
    }

    /**
     *
     * @return if the sensor is pressed
     */
    public boolean getTS() {
        return touchSensor.isPressed();
    }

    /**
     *
     * Sets the shooter to a specific angle
     *
     * @param targetAngle the angle for the shooter to go to (degrees)
     */
    public void setAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, ShooterConstants.MIN_ANGLE, ShooterConstants.MAX_ANGLE);

        this.targetPos = targetAngle;

        double power = pid.calculate(getPosition(), targetAngle);
        servo.setPower(power);
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



    public static ShooterSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        if (instance == null) {
            instance = new ShooterSubsystem(hardwareMap, gamepad1, gamepad2);
        }
        return instance;
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ShooterSubsystem not initialized. Call getInstance(hardwareMap, gamepad2) first.");
        }
        return instance;
    }
}