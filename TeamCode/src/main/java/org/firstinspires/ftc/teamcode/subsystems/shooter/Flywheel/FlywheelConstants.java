package org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConstants {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "ls";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "rs";

    public static double kP = 0.15;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0016;

    public static double target = 0;

    public static final double CLOSE_SP = 2350;
    public static final double FAR_SP = 2925;


}
