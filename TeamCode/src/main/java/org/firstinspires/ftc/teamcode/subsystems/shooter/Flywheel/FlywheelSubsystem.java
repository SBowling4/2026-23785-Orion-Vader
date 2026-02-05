package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.lib.orion.feedforward.FeedForward;
import org.firstinspires.ftc.lib.orion.hardware.OrionMotor;
import org.firstinspires.ftc.lib.orion.util.units.AngularVelocityUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotCalculator;


public class FlywheelSubsystem {
    public OrionMotor leftMotor;
    public OrionMotor rightMotor;

    public double lastTargetRPM = 0.0;
    public double lastTargetVolts = 0.0;

    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;
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
        leftMotor = new OrionMotor(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new OrionMotor(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setBrake(false);
        rightMotor.setBrake(false);

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setCoefficients(
                new PIDCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD
                ),
                new FeedForward.FeedForwardCoefficients(
                        FlywheelConstants.kS,
                        FlywheelConstants.kV,
                        FlywheelConstants.kA
                )
        );

        rightMotor.setCoefficients(
                new PIDCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD
                ),
                new FeedForward.FeedForwardCoefficients(
                        FlywheelConstants.kS,
                        FlywheelConstants.kV,
                        FlywheelConstants.kA
                )
        );
    }

    /**
     * Main loop for the Flywheel Subsystem
     */
    public void loop() {
        leftMotor.setCoefficients(
                new PIDCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD
                ),
                new FeedForward.FeedForwardCoefficients(
                        FlywheelConstants.kS,
                        FlywheelConstants.kV,
                        FlywheelConstants.kA
                )
        );

        rightMotor.setCoefficients(
                new PIDCoefficients(
                        FlywheelConstants.kP,
                        FlywheelConstants.kI,
                        FlywheelConstants.kD
                ),
                new FeedForward.FeedForwardCoefficients(
                        FlywheelConstants.kS,
                        FlywheelConstants.kV,
                        FlywheelConstants.kA
                )
        );

        if (Robot.tuningMode) {
            setVelocity(FlywheelConstants.target);
        } else {
            if (gamepad1.right_bumper) {
                setVelocityFromDistance();
            } else {
                setPower(.2);
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
        return leftMotor.getVelocity(AngularVelocityUnit.RPM, new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME));
    }

    /**
     * Sets the target velocity for the flywheel using a combination of feedforward and PID control
     *
     * @param targetRPM Target velocity (rpm)
     */
    public void setVelocity(double targetRPM) {
        lastTargetRPM = targetRPM;

        leftMotor.setVelocity(targetRPM, AngularVelocityUnit.RPM, new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME));
        rightMotor.setVelocity(targetRPM, AngularVelocityUnit.RPM, new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME));
    }

    public void setVelocityFromDistance() {
        lastTargetRPM = ShotCalculator.getInstance().getShootingParameters().flywheelRPM();

        leftMotor.setVelocity(lastTargetRPM, AngularVelocityUnit.RPM, new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME));
        rightMotor.setVelocity(lastTargetRPM, AngularVelocityUnit.RPM, new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME));
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

        return Math.abs(lastTargetRPM - getVelocity()) < 100;
    }



    public double findVelocity() {
        return ShotCalculator.getInstance().getShootingParameters().flywheelRPM();
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
        packet.put("Flywheel/Velocity from Distance", ShotCalculator.getInstance().getShootingParameters().flywheelRPM());
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