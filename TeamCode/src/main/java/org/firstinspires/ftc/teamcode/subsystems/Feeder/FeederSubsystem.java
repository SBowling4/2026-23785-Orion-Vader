package org.firstinspires.ftc.teamcode.subsystems.Feeder;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.KICKER_STATE;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.STOPPER_STATE;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants.FEEDER_STATE;



@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class FeederSubsystem {
    private MotorEx feederMotor;
    private ServoImplEx kickerServo;
    private ServoImplEx stopperServo;

    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Telemetry telemetry;

    private static FeederSubsystem instance;

    private STOPPER_STATE stopperState;
    private KICKER_STATE kickerState;
    private FEEDER_STATE feederState;

    /**
     * Feeder Subsystem constructor
     */
    private FeederSubsystem(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.telemetry = telemetry;
    }

    /**
     * Initializes the Feeder Subsystem
     */
    public void init() {
        feederMotor = new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME);

        kickerServo = hardwareMap.get(ServoImplEx.class, FeederConstants.KICKER_SERVO_NAME);
        stopperServo = hardwareMap.get(ServoImplEx.class, FeederConstants.STOPPER_SERVO_NAME);

        kickerState = KICKER_STATE.OUT;
        stopperState = STOPPER_STATE.OPEN;
        feederState = FEEDER_STATE.STOP;

//        flywheelSubsystem = FlywheelSubsystem.getInstance();
//        shooterSubsystem = ShooterSubsystem.getInstance();
    }

    /**
     * Main loop for the Feeder Subsystem
     */
    public void loop() {
//        if (gamepad1.a && (gamepad1.left_bumper || gamepad1.right_bumper)) {
//            autoFeed();
/*        } else*/ if (gamepad1.a) {
            setFeederState(FEEDER_STATE.IN);
        } else if (gamepad1.y || gamepad1.b) {
            setFeederState(FEEDER_STATE.OUT);
        } else {
            setFeederState(FEEDER_STATE.STOP);
        }

        setTelemetry();
    }

    /**
     * Automatically feeds objects when the flywheel and shooter are at their target states.
     */
    public void autoFeed() {
//        if (flywheelSubsystem.atVelocity() && shooterSubsystem.atPosition()) {
//            feed();
//        } else {
//            stop();
//        }
    }

    public void setFeederState(FEEDER_STATE state) {
        this.feederState = state;

        feederMotor.set(state.getPower());
    }

    public void setStopperState(STOPPER_STATE state) {
        this.stopperState = state;

        stopperServo.setPosition(state.getPosition());
    }

    public void setKickerState(KICKER_STATE state) {
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

    private void setTelemetry() {
        telemetry.addLine("//Feeder//");
        telemetry.addData("Feeder State", feederState.toString());
        telemetry.addData("Kicker State", kickerState.toString());
        telemetry.addData("Stopper State", stopperState.toString());
        telemetry.addLine();
    }

    /**
     * Singleton pattern to get the instance of FeederSubsystem.
     * @param hardwareMap The hardware map to initialize the subsystem.
     * @param gamepad1 The gamepad to control the subsystem.
     *                 prepare for BDR
     */
    public static FeederSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        if (instance == null) {
            instance = new FeederSubsystem(hardwareMap, gamepad1, telemetry);
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


}
