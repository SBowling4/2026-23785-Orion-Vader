package org.firstinspires.ftc.lib.orion.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.lib.orion.feedforward.FeedForward;
import org.firstinspires.ftc.lib.orion.feedforward.FeedForwardCoefficients;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;

public class Motor {
    private final MotorEx internalMotor;
    private final com.arcrobotics.ftclib.hardware.motors.Motor.Encoder encoder;
    private final String name;

    private PIDController pidController;
    private FeedForward feedForward;

    public double lastAppliedVoltage = 0;

    public static final double TICKS_PER_REVOLUTION = 25;

    public Motor(HardwareMap hardwareMap, String name) {
        this.name = name;

        internalMotor = new MotorEx(hardwareMap, name);
        encoder = internalMotor.encoder;

    }

    public Encoder getEncoder() {
        return encoder;
    }
    public void resetEncoder() {
        internalMotor.resetEncoder();
    }

    public void setInverted(boolean isInverted) {
        internalMotor.setInverted(isInverted);
    }

    public void setBrake(boolean shouldBrake) {
        if (shouldBrake) {
            internalMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            internalMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    public double getVelocity() {
        return (internalMotor.getVelocity() / Motor.TICKS_PER_REVOLUTION) / 60.0;
    }

    public void setPIDFCoefficients(PIDCoefficients pidCoefficients) {
        pidController = new PIDController(
                pidCoefficients.p,
                pidCoefficients.i,
                pidCoefficients.d
        );
    }

    public void setFeedForwardCoefficients(FeedForwardCoefficients feedForwardCoefficients) {
        feedForward = new FeedForward(feedForwardCoefficients);
    }

    public void setCoefficients(PIDCoefficients pidCoefficients, FeedForwardCoefficients feedForwardCoefficients) {
        pidController = new PIDController(
                pidCoefficients.p,
                pidCoefficients.i,
                pidCoefficients.d
        );

        feedForward = new FeedForward(feedForwardCoefficients);
    }

    public void setPower(double power) {
        internalMotor.set(power);
    }

    public void setVoltage(double volts) {
        lastAppliedVoltage = volts;
        setPower(volts / Robot.getRobotVoltage());
    }

    public void stop() {
        internalMotor.stopMotor();
    }

    public void setVelocity(double targetVelocity, double currentVelocity) {
        if (pidController == null || feedForward == null) {
            throw new IllegalStateException("PIDController and FeedForward must be set before using setVelocity.");
        }

        double pidOutput = pidController.calculate(currentVelocity, targetVelocity);
        double ffOutput = feedForward.calculate(targetVelocity);

        double totalOutput = pidOutput + ffOutput;

        setVoltage(totalOutput);
    }

    public void setVelocity(double targetVelocity) {
        if (pidController == null || feedForward == null) {
            throw new IllegalStateException("PIDController and FeedForward must be set before using setVelocity.");
        }

        double pidOutput = pidController.calculate(internalMotor.getVelocity(), targetVelocity);
        double ffOutput = feedForward.calculate(targetVelocity);

        double totalOutput = pidOutput + ffOutput;

        setVoltage(totalOutput);
    }

    public MotorEx getInternalMotor() {
        return internalMotor;
    }

    public String getName() {
        return this.name;
    }

}
