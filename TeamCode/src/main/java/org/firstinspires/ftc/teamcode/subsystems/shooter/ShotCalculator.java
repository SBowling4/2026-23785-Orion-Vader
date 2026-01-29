package org.firstinspires.ftc.teamcode.subsystems.shooter;


import org.firstinspires.ftc.lib.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

public class ShotCalculator {
    private static ShotCalculator instance = null;

    private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();
    public record ShootingParameters(double flywheelRPM, double hoodAngle) {}

    static {
        hoodMap.put(0.0,0.0);

        flywheelMap.put(0.0,0.0);
    }

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
