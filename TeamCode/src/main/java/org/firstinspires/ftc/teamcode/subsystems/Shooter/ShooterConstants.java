package org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;

public class ShooterConstants {
    public static final String SERVO_NAME = "angle";
    public static final String TOUCH_SENSOR_NAME = "ts";
    public static double kP = 0.0649;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static final double GEAR_RATIO = .5;
    public static final double MIN_ANGLE = 0.0;
    public static final double MAX_ANGLE = 25.0;

    public static final double FAR_ANGLE = 25;
    public static final double MID_ANGLE = 20.5;
    public static final double CLOSE_ANGLE = 6;

    public static final double CLOSE_AUTO_ANGLE = 24;

}
