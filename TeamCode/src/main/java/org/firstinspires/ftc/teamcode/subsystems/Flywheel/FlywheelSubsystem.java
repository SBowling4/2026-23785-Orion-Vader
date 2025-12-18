package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.FeedForward;


public class FlywheelSubsystem {
    public MotorEx leftMotor;
    public MotorEx rightMotor;

    private FeedForward ff;
    private PIDController pid;
    public double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public double tuningVelocity = 0.0;

    private static FlywheelSubsystem instance;

    public FlywheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        leftMotor = new MotorEx(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        ff = new FeedForward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

        pid = new PIDController(FlywheelConstants.kP, FlywheelConstants.kI, FlywheelConstants.kD);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {
        if (gamepad1.left_bumper) {
            setPower(1);
        } else if (gamepad1.right_bumper) {
            setPower(.5);
        } else if (gamepad1.a || gamepad1.y) {
            setPower(-1);
        } else {
            stop();
        }
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public double getVelocity() {
        return -(leftMotor.getVelocity()  / FlywheelConstants.TICKS_PER_REVOLUTION) * 2 * Math.PI;
    }

    public void setVelocity(double targetRadPerSec) {
        double currentRadPerSec = getVelocity();

        lastTargetRadPerSec = targetRadPerSec;

        // Use absolute value for feedforward to ensure kS term has correct sign
        // The direction will be handled by the overall voltage sign
        double ffVolts = ff.calculate(Math.abs(targetRadPerSec));

        // PID works on signed values
        double pidOutput = pid.calculate(currentRadPerSec, targetRadPerSec);

        // Combine feedforward and feedback, applying correct sign
        double volts = Math.signum(targetRadPerSec) * ffVolts + pidOutput;

        lastTargetVolts = volts;

        setVoltage(-volts);
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - lastTargetRadPerSec) < 5;
    }



    public void setVoltage(double volts) {
        double power = Range.clip(volts / Robot.getRobotVoltage(), -1.0, 1.0);
        leftMotor.set(power);
        rightMotor.set(power);
    }

    /**
     * Equation obtained from here: <a href="https://docs.google.com/spreadsheets/d/1m6Tb_BewsEm0vuEWVIr-rKV5Jfy468Ui95xVuQbh-_I/edit?usp=sharing">Spreadsheet</a>
     *
     * @param distance distance (m) from target (Front of robot to base of goal)
     * @return Desired velocity for flywheel (rad/s)
     */
    public double findVelocity(double distance) {
        return 204 + 74.4 * distance + -23.8 * Math.pow(distance, 2) + 5.36 * Math.pow(distance, 3);
    }

    public void setPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }


    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static FlywheelSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("FlywheelSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }


}