package org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotCalculator;

public class HoodSubsystem {
    public ServoImplEx hoodServo;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private static HoodSubsystem instance;

    public double position =  0;

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
        hoodServo = hardwareMap.get(ServoImplEx.class, HoodConstants.SERVO_NAME);
    }

    /**
     * Loops the Shooter Subsystem
     */
    public void loop() {
        if (Robot.tuningMode) {
            setAngle(HoodConstants.target);
        } else {
            setAngleFromDistance();
        }
    }


    public void setAngle(double targetAngle) {
        targetAngle = Range.clip(targetAngle, HoodConstants.HIGHEST_ANGLE, HoodConstants.LOWEST_ANGLE);

        this.position = targetAngle;
        hoodServo.setPosition(targetAngle);
    }

    public void setAngleFromDistance() {
        setAngle(ShotCalculator.getInstance().getShootingParameters().hoodAngle());
    }


    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Hood/Position", position);
        packet.put("Hood/Angle from Distance", ShotCalculator.getInstance().getShootingParameters().hoodAngle());
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