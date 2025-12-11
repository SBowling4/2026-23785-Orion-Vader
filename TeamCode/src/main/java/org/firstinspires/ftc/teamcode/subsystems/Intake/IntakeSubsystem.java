package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class IntakeSubsystem {

    private static IntakeSubsystem instance;

    MotorEx intakeMotor;

    HardwareMap hardwareMap;

    Gamepad gamepad1;

    public IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        intakeMotor = new MotorEx(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);
    }

    public void loop() {
        if (gamepad1.a || gamepad1.b) {
            intake();
        } else if (gamepad1.y){
            out();
        } else {
            stop();
        }
    }
    public void intake() {
        intakeMotor.set(1);
    }

    public void out() {
        intakeMotor.set(-1);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public static IntakeSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new IntakeSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("IntakeSubsystem not initialized. Call getInstance(hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}
