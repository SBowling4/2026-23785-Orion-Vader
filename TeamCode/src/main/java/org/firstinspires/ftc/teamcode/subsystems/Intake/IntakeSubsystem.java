package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.INTAKE_STATE;


import org.firstinspires.ftc.teamcode.Robot;

public class IntakeSubsystem {
    private MotorEx intakeMotor;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    private INTAKE_STATE intakeState;

    private static IntakeSubsystem instance;



    /**
     * Intake Subsystem constructor
     */
    private IntakeSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the Intake Subsystem
     */
    public void init() {
        intakeMotor = new MotorEx(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);

        intakeState = INTAKE_STATE.STOP;
    }

    /**
     * Main loop for the Intake Subsystem
     */
    public void loop() {
        if (gamepad1.a || gamepad1.b) {
            setState(INTAKE_STATE.INTAKE);
        } else if (gamepad1.y){
            setState(INTAKE_STATE.OUT);
        } else {
            setState(INTAKE_STATE.STOP);
        }

        setTelemetry();
    }

    public void setState(INTAKE_STATE state) {
        this.intakeState = state;
        intakeMotor.set(state.getPower());
    }

    public INTAKE_STATE getState() {
        return this.intakeState;
    }

    private void setTelemetry() {
        telemetry.addLine("//Intake//");
        telemetry.addData("Intake State", intakeState.toString());
        telemetry.addLine();
    }


    /**
     * Singleton pattern to get the instance of IntakeSubsystem.
     */
    public static IntakeSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new IntakeSubsystem(hardwareMap, gamepad1, telemetry);
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
