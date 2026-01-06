package org.firstinspires.ftc.teamcode.subsystems.Hood;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;

public class HoodSubsystem {
    public CRServoImplEx hoodServo;
    public Motor.Encoder hoodEncoder;
    public PIDFController pid;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private static HoodSubsystem instance;

    public double tuningPos = 0;
    public double targetPos = 0;

    /**
     * Shooter Subsystem constructor
     */
    public HoodSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    /**
     * Initializes the Shooter Subsystem
     */
    public void init() {
        hoodServo = hardwareMap.get(CRServoImplEx.class, HoodConstants.SERVO_NAME);

        hoodEncoder = new MotorEx(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME).encoder;

        pid = new PIDFController(
                HoodConstants.kP,
                HoodConstants.kI,
                HoodConstants.kD,
                HoodConstants.kF
        );

        double ticksPerRev = 8192;
        double degreesPerPulse = (360.0 * HoodConstants.GEAR_RATIO) / ticksPerRev;

        hoodEncoder.setDistancePerPulse(degreesPerPulse);

        hoodEncoder.reset();

        pid.setTolerance(1);


        tuningPos = 0;
    }

    /**
     * Loops the Shooter Subsystem
     */
    public void loop() {
//        if (Robot.tuningMode) {
//            if (gamepad1.dpad_up) {
//                tuningPos += .5;
//            } else if (gamepad1.dpad_down) {
//                tuningPos -= .5;
//            }
//
//            tuningPos = Range.clip(tuningPos, 0, 25);
//
//            setAngle(tuningPos);
//      }

        if (gamepad1.right_bumper) {
            setAngle(17);
        } else if (gamepad1.left_bumper) {
            setAngle(3);
        } else {
            setAngle(0);
        }
    }

    /**
     *
     * @return if the shooter is at the target position
     */
    public boolean atPosition() {
        return Math.abs(getPosition() - targetPos) < 1;
    }



    /**
     *
     * @return the position of the shooter (degrees)
     */
    public double getPosition() {
        int ticksPerRev = 8192;
        double revolutions = (double) hoodEncoder.getPosition() / ticksPerRev;

        return revolutions * 360.0 * HoodConstants.GEAR_RATIO;
    }


    /**
     *
     * Sets the shooter to a specific angle
     *
     * @param targetAngle the angle for the shooter to go to (degrees)
     */
    public void setAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, HoodConstants.MIN_ANGLE, HoodConstants.MAX_ANGLE);

        targetPos = targetAngle;

        double power = pid.calculate(getPosition(), targetAngle);
        hoodServo.setPower(power);
    }


    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Hood//");
        telemetry.addData("Position", getPosition());
        telemetry.addData("Target", targetPos);
        telemetry.addLine();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Hood/Position", getPosition());
        packet.put("Hood/Target", targetPos);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


    /**
     * Singleton pattern to get the instance of the ShooterSubsystem
     *
     * @param gamepad1    Gamepad1 from the OpMode
     * @return Instance of the ShooterSubsystem
     */
    public static HoodSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new HoodSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    /**
     * Singleton pattern to get the instance of the ShooterSubsystem
     *
     * @return Instance of the ShooterSubsystem
     */
    public static HoodSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("ShooterSubsystem not initialized. Call getInstance(hardwareMap, gamepad2) first.");
        }
        return instance;
    }
}