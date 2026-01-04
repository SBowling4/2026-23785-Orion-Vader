package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.INTAKE_STATE;

public class IntakeSubsystem {
    private Motor intakeMotor;

    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;

    private INTAKE_STATE intakeState;

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
        intakeMotor = new Motor(hardwareMap, IntakeConstants.INTAKE_MOTOR_NAME);

        intakeState = INTAKE_STATE.STOP;
    }

    /**
     * Main loop for the Intake Subsystem
     */
    public void loop() {
        if (gamepad1.a) {
            setState(INTAKE_STATE.INTAKE);
        } else if (gamepad1.y){
            setState(INTAKE_STATE.OUT);
        } else {
            setState(INTAKE_STATE.STOP);
        }
    }

    public void setState(INTAKE_STATE state) {
        this.intakeState = state;
        intakeMotor.setPower(state.getPower());
    }

    public INTAKE_STATE getState() {
        return this.intakeState;
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Intake//");
        telemetry.addData("Intake State", intakeState.toString());
        telemetry.addLine();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Intake/IntakeState", intakeState.toString());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
