package org.firstinspires.ftc.teamcode.subsystems.shooter.Hood;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class HoodConstants {
    public static final String SERVO_NAME = "hood";
    public static final double HIGHEST_ANGLE = .14;
    public static final double LOWEST_ANGLE = .7;

    public static double kRapidFire = -.002;

    public static double target = 0;
}
