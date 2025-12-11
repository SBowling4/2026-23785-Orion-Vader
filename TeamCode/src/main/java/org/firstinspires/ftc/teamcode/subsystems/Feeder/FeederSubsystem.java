package org.firstinspires.ftc.teamcode.subsystems.Feeder;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;

public class FeederSubsystem {
    private MotorEx feederMotor;
    private FlywheelSubsystem flywheelSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private static FeederSubsystem instance;


    public FeederSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        feederMotor = new MotorEx(hardwareMap, FeederConstants.FEEDER_MOTOR_NAME);

        flywheelSubsystem = FlywheelSubsystem.getInstance();
        shooterSubsystem = ShooterSubsystem.getInstance();
    }

    public void loop() {
        if (gamepad1.a && (gamepad1.left_bumper || gamepad1.right_bumper)) {
            autoFeed();
        } else if (gamepad1.a) {
            feed();
        } else if (gamepad1.y || gamepad1.b) {
            back();
        } else {
            stop();
        }
    }

    public void feed() {
        feederMotor.set(1);
    }

    public void feed(double power) {
        feederMotor.set(power);
    }

    public void autoFeed() {
        if (flywheelSubsystem.atVelocity() && shooterSubsystem.atPosition()) {
            feed();
        } else {
            stop();
        }
    }

    public void back() {
        feederMotor.set(-1);
    }


    public void stop() {
        feederMotor.stopMotor();
    }


    public static FeederSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FeederSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static FeederSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Call getInstance(HardwareMap hardwareMap) first");
        }
        return instance;
    }


}
