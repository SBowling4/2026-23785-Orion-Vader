package org.firstinspires.ftc.lib.orion.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class OrionMotor {
    private final MotorEx internalMotor;
    private final com.arcrobotics.ftclib.hardware.motors.Motor.Encoder encoder;
    private final String name;

    private PIDFController pidfController;
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

    /**
     * Gets the velocity in revolutions per second.
     */
    public double getVelocity() {
        return (internalMotor.getVelocity() / OrionMotor.TICKS_PER_REVOLUTION) / 60.0;
    }


    /**
     * Sets the PIDF coefficients for velocity control.
     *
     * @param pidfCoefficients the PIDF coefficients
     */
    public void setCoefficients(PIDFCoefficients pidfCoefficients) {
        pidfController = new PIDFController(
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f
        );
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
     * @param currentVelocity the current velocity
     */
    public void setVelocity(double targetVelocity, double currentVelocity) {
        if (pidfController == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidfController.calculate(currentVelocity, targetVelocity);

        setVoltage(pidOutput);
    }

    /**
     * Sets the target velocity using PIDF control.
     *
     * @param targetVelocity the desired velocity
     */
    public void setVelocity(double targetVelocity) {
        if (pidfController == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidfController.calculate(internalMotor.getVelocity(), targetVelocity);

        setVoltage(pidOutput);
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
