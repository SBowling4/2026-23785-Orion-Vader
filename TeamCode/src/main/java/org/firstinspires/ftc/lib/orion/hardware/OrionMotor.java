package org.firstinspires.ftc.lib.orion.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.feedforward.FeedForward;
import org.firstinspires.ftc.lib.orion.util.units.AngularVelocityUnit;
import org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class OrionMotor {
    private final MotorEx internalMotor;
    private final com.arcrobotics.ftclib.hardware.motors.Motor.Encoder encoder;
    private final String name;

    private PIDController pidController;
    private FeedForward feedForward;
    public double lastAppliedVoltage = 0;

    public static final double TICKS_PER_REVOLUTION = 28;

    public OrionMotor(HardwareMap hardwareMap, String name) {
        this.name = name;

        internalMotor = new MotorEx(hardwareMap, name);
        encoder = internalMotor.encoder;

    }

    /**
     * Gets the motor encoder.
     *
     * @return the encoder
     */
    public Encoder getEncoder() {
        return encoder;
    }
    /**
     * Resets the motor encoder to zero.
     */
    public void resetEncoder() {
        internalMotor.resetEncoder();
    }

    /**
     * Sets whether the motor direction is inverted.
     *
     * @param isInverted true to invert, false for normal
     */
    public void setInverted(boolean isInverted) {
        internalMotor.setInverted(isInverted);
    }

    /**
     * Sets whether the motor should brake or coast when set to zero power.
     *
     * @param shouldBrake true to brake, false to coast
     */
    public void setBrake(boolean shouldBrake) {
        if (shouldBrake) {
            internalMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            internalMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    public double getVelocity(AngularVelocityUnit unit) {
        double ticksPerSecond = internalMotor.getVelocity();

        return switch (unit) {
            case TICKSPERSECOND -> ticksPerSecond;

            case RPM ->
                    (ticksPerSecond / OrionMotor.TICKS_PER_REVOLUTION) * 60.0;

            case RADPERSECOND ->
                    (ticksPerSecond / OrionMotor.TICKS_PER_REVOLUTION) * 2.0 * Math.PI;

            default ->
                    throw new IllegalArgumentException(
                            "Unsupported AngularVelocityUnit: " + unit
                    );
        };
    }



    public double getVelocity(AngularVelocityUnit unit, MotorEx altMotor) {
        double ticksPerSecond = altMotor.getVelocity();

        return switch (unit) {
            case TICKSPERSECOND -> ticksPerSecond;

            case RPM ->
                    (ticksPerSecond / OrionMotor.TICKS_PER_REVOLUTION) * 60.0;

            case RADPERSECOND ->
                    (ticksPerSecond / OrionMotor.TICKS_PER_REVOLUTION) * 2.0 * Math.PI;

            default ->
                    throw new IllegalArgumentException(
                            "Unsupported AngularVelocityUnit: " + unit
                    );
        };
    }



    /**
     * Sets the PID and FeedForward coefficients for velocity control.
     *
     * @param pidCoefficients the PID coefficients
     *                        @param ffCoefficients  the FeedForward coefficients
     */
    public void setCoefficients(PIDCoefficients pidCoefficients, FeedForward.FeedForwardCoefficients ffCoefficients) {
        pidController = new PIDController(
                pidCoefficients.p,
                pidCoefficients.i,
                pidCoefficients.d
        );

        feedForward = new FeedForward(ffCoefficients);
    }

    /**
     * Sets the motor power.
     *
     * @param power the power level (-1.0 to 1.0)
     */
    public void setPower(double power) {
        internalMotor.set(power);
    }

    /**
     * Sets the motor voltage.
     *
     * @param volts the voltage to apply
     */
    public void setVoltage(double volts) {
        lastAppliedVoltage = volts;
        setPower(volts / Robot.getRobotVoltage());
    }
    /**
     * Stops the motor.
     */

    public void stop() {
        internalMotor.stopMotor();
    }

    /**
     * Sets the target velocity using PIDF control.
     *
     * @param targetVelocity  the desired velocity
     */
    public void setVelocity(double targetVelocity, AngularVelocityUnit unit) {
        if (pidController == null || feedForward == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidController.calculate(getVelocity(unit), targetVelocity);
        double ffOutput = feedForward.calculate(targetVelocity);

        setVoltage(pidOutput + ffOutput);
    }

    /**
     * Sets the target velocity using PIDF control.
     *
     * @param targetVelocity  the desired velocity
     */
    public void setVelocity(double targetVelocity, AngularVelocityUnit unit, MotorEx altMotor) {
        if (pidController == null || feedForward == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidController.calculate(getVelocity(unit, altMotor), targetVelocity);
        double ffOutput = feedForward.calculate(targetVelocity);

        setVoltage(pidOutput + ffOutput);
    }

    /**
     * Gets the internal MotorEx instance.
     *
     * @return the internal MotorEx
     */
    public MotorEx getInternalMotor() {
        return internalMotor;
    }

    /**
     * Gets the name of the motor.
     *
     * @return the motor name
     */
    public String getName() {
        return this.name;
    }

}
