package org.firstinspires.ftc.teamcode.subsystems.Flywheel;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.lib.orion.feedforward.FeedForwardCoefficients;
import org.firstinspires.ftc.lib.orion.hardware.Motor;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlywheelSubsystem {
    public Motor leftMotor;
    public Motor rightMotor;

    public double lastTargetRadPerSec = 0.0;

    public double lastTargetVolts = 0.0;

    private final Gamepad gamepad1;
    private final HardwareMap hardwareMap;

    public double tuningVelocity = 0.0;

    private static FlywheelSubsystem instance;

    /**
     * Flywheel Subsystem constructor
     */
    private FlywheelSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    /**
     * Initializes the Flywheel Subsystem
     */
    public void init() {
        leftMotor = new Motor(hardwareMap, FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME);
        rightMotor = new Motor(hardwareMap, FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_NAME);

        leftMotor.resetEncoder();
        rightMotor.resetEncoder();

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setBrake(false);
        rightMotor.setBrake(false);

        leftMotor.setCoefficients(new PIDCoefficients(
                    FlywheelConstants.kP,
                    FlywheelConstants.kI,
                    FlywheelConstants.kD
                ), new FeedForwardCoefficients(
                    FlywheelConstants.kS,
                    FlywheelConstants.kV
                ));

        rightMotor.setCoefficients(new PIDCoefficients(
                    FlywheelConstants.kP,
                    FlywheelConstants.kI,
                    FlywheelConstants.kD
            ), new FeedForwardCoefficients(
                    FlywheelConstants.kS,
                    FlywheelConstants.kV
            ));
    }

    /**
     * Main loop for the Flywheel Subsystem
     */
    public void loop() {
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            setPower(1);
        } else {
            stop();
        }
    }

    /**
     * Stops the flywheel motors
     */
    public void stop() {
        leftMotor.stop();
        rightMotor.stop();
    }

    /**
     * Gets the current velocity of the flywheel in radians per second
     *
     * @return Current velocity (RPM)
     */
    public double getVelocity() {
        return -(leftMotor.getVelocity()  / FlywheelConstants.TICKS_PER_REVOLUTION) / 60.0;
    }

    /**
     * Sets the target velocity for the flywheel using a combination of feedforward and PID control
     *
     * @param targetRadPerSec Target velocity (rad/s)
     */
    public void setVelocity(double targetRadPerSec) {
        leftMotor.setVelocity(targetRadPerSec);
        rightMotor.setVelocity(targetRadPerSec);
    }

    /**
     * Checks if the flywheel is at the target velocity
     *
     * @return true if at target velocity, false otherwise
     */
    public boolean atVelocity() {
        return Math.abs(getVelocity() - lastTargetRadPerSec) < 5;
    }


    /**
     * Equation obtained from here: <a href="https://docs.google.com/spreadsheets/d/1m6Tb_BewsEm0vuEWVIr-rKV5Jfy468Ui95xVuQbh-_I/edit?usp=sharing">Spreadsheet</a>
     *
     * @param distance distance (m) from target (Front of robot to base of goal)
     * @return Desired velocity for flywheel (rad/s)
     */
    public double findVelocity(double distance) {
        return 204 + 74.4 * distance + -23.8 * Math.pow(distance, 2) + 5.36 * Math.pow(distance, 3);
    }

    /**
     * Sets the power for both flywheel motors
     *
     * @param power Power value (-1.0 to 1.0)
     */
    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Flywheel//");
        telemetry.addData("Flywheel Velocity", getVelocity());
        telemetry.addData("Flywheel Target", lastTargetRadPerSec);
        telemetry.addData("Flywheel Volts", lastTargetVolts);
        telemetry.addLine();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Flywheel/Velocity", getVelocity());
        packet.put("Flywheel/Target", lastTargetRadPerSec);
        packet.put("Flywheel/Volts", lastTargetVolts);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }

    /**
     * Singleton pattern to get the instance of FlywheelSubsystem
     *
     * @param gamepad1    Gamepad for user input
     * @return Instance of FlywheelSubsystem
     */
    public static FlywheelSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new FlywheelSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    /**
     * Singleton pattern to get the instance of FlywheelSubsystem
     *
     * @return Instance of FlywheelSubsystem
     */
    public static FlywheelSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("FlywheelSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }


}