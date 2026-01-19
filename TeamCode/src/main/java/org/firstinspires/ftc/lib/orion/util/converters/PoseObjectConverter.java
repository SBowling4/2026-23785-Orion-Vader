package org.firstinspires.ftc.lib.orion.util.converters;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * PoseConverter is a utility class for converting between different pose representations.
 * No coordinate system conversion, just object conversion.
 */
public class PoseObjectConverter {
    public static Pose3d ftcToWPILib(Pose3D pose3D) {
        return new Pose3d(
                pose3D.getPosition().x,
                pose3D.getPosition().y,
                0,
                new Rotation3d(Rotation2d.fromDegrees(pose3D.getOrientation().getYaw(AngleUnit.RADIANS)))
        );
    }

    public static Pose3D wpiLibToFTC(Pose3d pose3d) {
        Position pos = new Position(DistanceUnit.METER, pose3d.getTranslation().getX(), pose3d.getTranslation().getY(), pose3d.getTranslation().getZ(), 0);
        YawPitchRollAngles angles = new YawPitchRollAngles(AngleUnit.RADIANS, pose3d.getRotation().toRotation2d().getRadians(), 0, 0, 0);
        return new Pose3D(
                pos,
                angles
        );
    }

}
