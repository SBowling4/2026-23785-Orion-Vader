package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConstants {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "ls";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "rs";

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 1.4;
    public static double kV = 0.00327;
    public static double kA = 0.0;

    public static double target = 0;

    public static final double CLOSE_SP = 2350;
    public static final double FAR_SP = 2925;


}
