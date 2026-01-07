package org.firstinspires.ftc.teamcode.subsystems.Flywheel;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConstants {
    public static final String LEFT_FLYWHEEL_MOTOR_NAME = "ls";
    public static final String RIGHT_FLYWHEEL_MOTOR_NAME = "rs";

    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double kS = 1.753;
    public static double kV = .0024;

    public static double target = 0;


}
