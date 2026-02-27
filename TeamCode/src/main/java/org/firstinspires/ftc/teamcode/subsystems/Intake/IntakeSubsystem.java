package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.hardware.OrionMotor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.intakeState;

public class IntakeSubsystem {
    private OrionMotor intakeMotor;

    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;

    private intakeState intakeState;

    private static IntakeSubsystem instance;



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
        intakeMotor = new OrionMotor(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);

        intakeState = IntakeConstants.intakeState.STOP;
    }

    /**
     * Main loop for the Intake Subsystem
     */
    public void loop() {
        if (Robot.tuningMode) {
            setState(IntakeConstants.intakeState.INTAKE);
        } else {
            if (gamepad1.a) {
                setState(IntakeConstants.intakeState.INTAKE);
            } else if (gamepad1.y){
                setState(IntakeConstants.intakeState.OUT);
            } else {
                setState(IntakeConstants.intakeState.STOP);
            }
        }

    }

    public void setState(intakeState state) {
        this.intakeState = state;
        intakeMotor.setPower(state.getPower());
    }

    public intakeState getState() {
        return this.intakeState;
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Intake/IntakeState", intakeState.toString());
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
