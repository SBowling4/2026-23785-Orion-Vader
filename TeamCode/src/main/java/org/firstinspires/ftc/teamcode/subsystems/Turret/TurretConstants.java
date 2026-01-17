package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static double kP = 1.5;
    public static double kI = 0.0;
    public static double kD = 0.05;
    public static double kF = 0.02;

    public static final String TURRET_SERVO_NAME = "turret";
    public static final double GEAR_RATIO = 1.0 / 3.0;

    public static double target = 0;
}
