package org.firstinspires.ftc.teamcode.subsystems.shooter;


import org.firstinspires.ftc.lib.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

public class ShotCalculator {
    private static ShotCalculator instance = null;

    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

    static {
        hoodMap.put(1.14,.65);
        hoodMap.put(1.45, .4);
        hoodMap.put(2.0, .12);

        flywheelMap.put(1.14,1800.0);
        flywheelMap.put(1.45, 1850.0);
        flywheelMap.put(2.0, 2000.0);
        flywheelMap.put(2.4, 2150.0);
        flywheelMap.put(2.9, 2275.0);
        flywheelMap.put(3.24, 2390.0);
        flywheelMap.put(3.39, 2450.0);
        flywheelMap.put(3.8, 2525.0);
    }

    public record ShootingParameters(double flywheelRPM, double hoodAngle) {}

    private DriveSubsystem driveSubsystem;

     public static ShotCalculator getInstance() {
        if (instance == null) {
            instance = new ShotCalculator();
        }

        return instance;
     }

    public ShootingParameters getShootingParameters() {
         if (driveSubsystem == null) {
             driveSubsystem = DriveSubsystem.getInstance();
         }

        double distance = driveSubsystem.getDistanceToGoal();

        double hoodAngle = hoodMap.get(distance);
        double flywheelRPM = flywheelMap.get(distance);

        return new ShootingParameters(flywheelRPM, hoodAngle);
    }

}
