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

    /**
     * Intake Subsystem constructor
     */
    private IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    /**
     * Initializes the Intake Subsystem
     */
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);
    }

    /**
     * Main loop for the Intake Subsystem
     */
    public void loop() {
        if (gamepad1.a || gamepad1.b) {
            intake();
        } else if (gamepad1.y){
            out();
        } else {
            stop();
        }
    }

    /**
     * Activates the intake motor to intake objects.
     */
    public void intake() {
        intakeMotor.set(1);
    }

    /**
     * Activates the intake motor to expel objects.
     */
    public void out() {
        intakeMotor.set(-1);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * Singleton pattern to get the instance of IntakeSubsystem.
     */
    public static IntakeSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new IntakeSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    /**
     * Get the existing instance of IntakeSubsystem.
     */
    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("IntakeSubsystem not initialized. Call getInstance(hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}
