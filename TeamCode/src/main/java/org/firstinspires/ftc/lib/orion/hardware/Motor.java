package org.firstinspires.ftc.lib.orion.hardware;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Motor {
    private final MotorEx internalMotor;
    private final com.arcrobotics.ftclib.hardware.motors.Motor.Encoder encoder;
    private final String name;

    private PIDFController pidfController;
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


    public void setCoefficients(PIDFCoefficients pidfCoefficients) {
        pidfController = new PIDFController(
                pidfCoefficients.p,
                pidfCoefficients.i,
                pidfCoefficients.d,
                pidfCoefficients.f
        );
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
        if (pidfController == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidfController.calculate(currentVelocity, targetVelocity);

        setVoltage(pidOutput);
    }

    public void setVelocity(double targetVelocity) {
        if (pidfController == null) {
            throw new IllegalStateException("PIDFController must be set before using setVelocity.");
        }

        double pidOutput = pidfController.calculate(internalMotor.getVelocity(), targetVelocity);

        setVoltage(pidOutput);
    }

    public MotorEx getInternalMotor() {
        return internalMotor;
    }

    public String getName() {
        return this.name;
    }

}
