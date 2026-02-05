package org.firstinspires.ftc.teamcode.subsystems.Feeder;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.lib.orion.hardware.OrionMotor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.KICKER_STATE;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.STOPPER_STATE;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.FEEDER_STATE;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodSubsystem;


@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class FeederSubsystem {
    private OrionMotor feederMotor;
    public ServoImplEx kickerServo;
    private ServoImplEx stopperServo;

    private boolean atVelocity = false;

    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private static FeederSubsystem instance;

    private STOPPER_STATE stopperState;
    private KICKER_STATE kickerState;
    private FEEDER_STATE feederState;


    /**
     * Feeder Subsystem constructor
     */
    private FeederSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    /**
     * Initializes the Feeder Subsystem
     */
    public void init() {
        feederMotor = new OrionMotor(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME);

        kickerServo = hardwareMap.get(ServoImplEx.class, FeederConstants.KICKER_SERVO_NAME);
        stopperServo = hardwareMap.get(ServoImplEx.class, FeederConstants.STOPPER_SERVO_NAME);

        stopperServo.setPwmRange(new PwmControl.PwmRange(450, 2550));

        flywheelSubsystem = FlywheelSubsystem.getInstance();
    }

    /**
     * Main loop for the Feeder Subsystem
     */
    public void loop() {
        if (Robot.tuningMode) {
            setFeederState(FEEDER_STATE.IN);
            setStopperState(STOPPER_STATE.OPEN);
            setKickerState(KICKER_STATE.OUT);
        } else {
            if (gamepad1.right_bumper || gamepad1.right_trigger > .5) {
                if (flywheelSubsystem.atVelocity() && !atVelocity) {
                    atVelocity = true;
                }

                if (atVelocity) {
                    setFeederState(FEEDER_STATE.IN);
                } else {
                    setFeederState(FEEDER_STATE.STOP);
                }
            } else if (gamepad1.a) {
                atVelocity = false;
                setFeederState(FEEDER_STATE.IN);
            } else if (gamepad1.y) {
                atVelocity = false;
                setFeederState(FEEDER_STATE.OUT);
            } else {
                atVelocity = false;
                setFeederState(FEEDER_STATE.STOP);
            }

            if (gamepad1.left_bumper || gamepad1.y) {
                setKickerState(KICKER_STATE.IN);
            } else {
                setKickerState(KICKER_STATE.OUT);
            }

            if (gamepad1.right_bumper) {
                setStopperState(STOPPER_STATE.OPEN);
            } else {
                setStopperState(STOPPER_STATE.CLOSED);
            }
        }





    }

    public void setFeederState(FEEDER_STATE state) {
        this.feederState = state;

        feederMotor.setPower(state.getPower());
    }

    public void setStopperState(STOPPER_STATE state) {
        this.stopperState = state;

        stopperServo.setPosition(state.getPosition());
    }

    public void setKickerState(KICKER_STATE state) {
        if (this.kickerState == state) {
            return;
        }

        this.kickerState = state;

        kickerServo.setPosition(state.getPosition());
    }

    public STOPPER_STATE getStopperState() {
        return stopperState;
    }

    public KICKER_STATE getKickerState() {
        return kickerState;
    }

    public FEEDER_STATE getFeederState() {
        return feederState;
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Feeder/FeederState", feederState.toString());
        packet.put("Feeder/Kicker/KickerState", kickerState.toString());
        packet.put("Feeder/Kicker/KickerPos", kickerServo.getController().getServoPosition(0));
        packet.put("Feeder/Stopper/StopperState", stopperState.toString());
        packet.put("Feeder/Stopper/StopperPos", stopperServo.getController().getServoPosition(2));
    }

    /**
     * Singleton pattern to get the instance of FeederSubsystem.
     * @param gamepad1 The gamepad to control the subsystem.
     *                 prepare for BDR
     */
    public static FeederSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FeederSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    /**
     * Singleton pattern to get the instance of FeederSubsystem.
     */
    public static FeederSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Call getInstance(HardwareMap hardwareMap) first");
        }
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }


}
