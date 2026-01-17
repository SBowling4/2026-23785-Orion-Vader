package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.lib.orion.hardware.Motor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;


public class FlywheelSubsystem {
    public Motor leftMotor;
    public Motor rightMotor;

    public double lastTargetRPM = 0.0;

    public double lastTargetVolts = 0.0;

    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
    private DriveSubsystem driveSubsystem;

    public double tuningVelocity = 0.0;

    private static FlywheelSubsystem instance;

    /**
     * Flywheel Subsystem constructor
     */
    private FlywheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    /**
     * Initializes the Flywheel Subsystem
     */
    public void init() {
        leftMotor = new Motor(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new Motor(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setBrake(false);
        rightMotor.setBrake(false);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setCoefficients(
                new PIDFCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD,
                        FlywheelConstants.kF
                )
        );

        rightMotor.setCoefficients(
                new PIDFCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD,
                        FlywheelConstants.kF
                )
        );


        driveSubsystem = DriveSubsystem.getInstance();
    }

    /**
     * Main loop for the Flywheel Subsystem
     */
    public void loop() {
        if (Robot.tuningMode) {
            setVelocity(FlywheelConstants.target);
        } else {
            if (Robot.mode == Robot.RobotMode.VADER) {
                if (gamepad1.right_bumper) {
                    setVelocity(findVelocity(driveSubsystem.getDistanceToGoal()));
                } else {
                    setPower(.35);
                }
            } else if (Robot.mode == Robot.RobotMode.KAOS) {
                if (gamepad1.right_bumper) {
                    setVelocity(FlywheelConstants.CLOSE_SP);
                } else if (gamepad1.right_trigger > 50) {
                    setVelocity(FlywheelConstants.FAR_SP);
                } else {
                    setPower(.35);
                }
            }

        }

    }

    /**
     * Stops the flywheel motors
     */
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    /**
     * Gets the current velocity of the flywheel in revolutions per minute
     *
     * @return Current velocity (RPM)
     */
    public double getVelocity() {
        return (new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME).getVelocity() / Motor.TICKS_PER_REVOLUTION) * 60;
    }

    /**
     * Sets the target velocity for the flywheel using a combination of feedforward and PID control
     *
     * @param targetRPM Target velocity (rpm)
     */
    public void setVelocity(double targetRPM) {
        lastTargetRPM = targetRPM;

        leftMotor.setVelocity(targetRPM, getVelocity());
        rightMotor.setVelocity(targetRPM, getVelocity());
    }

    public void setVoltage(double volts) {
        lastTargetVolts = volts;

        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    /**
     * Checks if the flywheel is at the target velocity
     *
     * @return true if at target velocity, false otherwise
     */
    public boolean atVelocity() {
        if (lastTargetRPM < getVelocity()) {
            return true;
        }

        return Math.abs(lastTargetRPM - getVelocity()) < 150;
    }


    /**
     * Equation obtained from here: <a href="https://docs.google.com/spreadsheets/d/1m6Tb_BewsEm0vuEWVIr-rKV5Jfy468Ui95xVuQbh-_I/edit?usp=sharing">Spreadsheet</a>
     *
     * @param distance distance (m) from target (Center of robot to back corner of goal)
     * @return Desired velocity for flywheel (rad/s)
     */
    public double findVelocity(double distance) {
        return 1474 + 1106 * distance + -451 * Math.pow(distance, 2) + 70.6 * Math.pow(distance, 3);
    }

    /**
     * Sets the power for both flywheel motors
     *
     * @param power Power value (-1.0 to 1.0)
     */
    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Flywheel/Velocity", getVelocity());
        packet.put("Flywheel/Target", lastTargetRPM);
        packet.put("Flywheel/Volts", leftMotor.lastAppliedVoltage);
    }

    /**
     * Singleton pattern to get the instance of FlywheelSubsystem
     *
     * @param gamepad1    Gamepad for user input
     * @return Instance of FlywheelSubsystem
     */
    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    /**
     * Singleton pattern to get the instance of FlywheelSubsystem
     *
     * @return Instance of FlywheelSubsystem
     */
    public static FlywheelSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("FlywheelSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }


}