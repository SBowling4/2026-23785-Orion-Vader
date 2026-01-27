package org.firstinspires.ftc.lib.orion.util.converters;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@SuppressWarnings("unused")
public final class CoordinateSystemConverter {
    private CoordinateSystemConverter() {}

    /**
     * WPILib's field coordinate system has the origin in the blue right corner of the field, with +X
     * away from blue driver station, but the FTC SDK field coordinate system has the origin in the
     * center, with +X towards the audience.
     *
     * @param translation3d A {@link Translation3d} in the WPILib coordinate system.
     * @return A {@link Position} transformed to be in the FTC SDK Field Coordinate System in meters.
     */
    public static Position WPILibToFTCFieldCoordinates(Translation3d translation3d) {
        return new Position(
                DistanceUnit.METER,
                translation3d.getY() - Units.feetToMeters(6),
                Units.feetToMeters(6) - translation3d.getX(),
                translation3d.getZ(),
                0);
    }

    /**
     * @param position A {@link Position} in the FTC SDK Field Coordinate System.
     * @return A {@link Translation3d} transformed to be in the WPILib coordinate system.
     */
    public static Translation3d ftcFieldCoordinatesToWPILib(Position position) {
        double xMeters;
        double yMeters;
        double zMeters;
        switch (position.unit) {
            case METER:
                xMeters = position.x;
                yMeters = position.y;
                zMeters = position.z;
                break;
            case CM:
                xMeters = position.x / 100;
                yMeters = position.y / 100;
                zMeters = position.z / 100;
                break;
            case MM:
                xMeters = position.x / 1000;
                yMeters = position.y / 1000;
                zMeters = position.z / 1000;
                break;
            case INCH:
                xMeters = Units.inchesToMeters(position.x);
                yMeters = Units.inchesToMeters(position.y);
                zMeters = Units.inchesToMeters(position.z);
                break;

            default:
                throw new IllegalArgumentException("Position unit was null!");
        }
        return new Translation3d(
                Units.feetToMeters(6) - yMeters, xMeters + Units.feetToMeters(6), zMeters);
    }

    /**
     * WPILib uses a North-West-Up coordinate system, but the FTC SDK Robot Coordinate System is a
     * East-North-Up system.
     *
     * @param translation3d A {@link Translation3d} in the WPILib coordinate system.
     * @return A {@link Position} transformed to be in the FTC SDK Field Coordinate System in meters.
     */
    public static Position WPILibToRobotCoordinates(Translation3d translation3d) {
        return new Position(
                DistanceUnit.METER, -translation3d.getY(), translation3d.getX(), translation3d.getZ(), 0);
    }

    /**
     * WPILib uses a North-West-Up coordinate system, but the FTC SDK Robot Coordinate System is a
     * East-North-Up system.
     *
     * @param position A {@link Position} in the FTC SDK Robot Coordinate System.
     * @return A {@link Translation3d} transformed to be in the WPILib coordinate system.
     */
    public static Translation3d robotCoordinatesToWPILib(Position position) {
        double xMeters;
        double yMeters;
        double zMeters;
        switch (position.unit) {
            case METER:
                xMeters = position.x;
                yMeters = position.y;
                zMeters = position.z;
                break;
            case CM:
                xMeters = position.x / 100;
                yMeters = position.y / 100;
                zMeters = position.z / 100;
                break;
            case MM:
                xMeters = position.x / 1000;
                yMeters = position.y / 1000;
                zMeters = position.z / 1000;
                break;
            case INCH:
                xMeters = Units.inchesToMeters(position.x);
                yMeters = Units.inchesToMeters(position.y);
                zMeters = Units.inchesToMeters(position.z);
                break;
            default:
                throw new IllegalArgumentException("Position unit was null!");
        }

        return new Translation3d(yMeters, -xMeters, zMeters);
    }

    /**
     * WPILib uses a North-West-Up coordinate system, but the FTC SDK Robot Coordinate System is a
     * East-North-Up system.
     *
     * @param rotation3d A {@link Rotation3d} in the WPILib coordinate system.
     * @return {@link YawPitchRollAngles} transformed to be in the FTC SDK Robot Coordinate System, in
     *     radians.
     */
    public static YawPitchRollAngles WPILibToSDKRotation(Rotation3d rotation3d) {
        return new YawPitchRollAngles(
                AngleUnit.RADIANS, rotation3d.getZ(), rotation3d.getY(), -rotation3d.getX(), 0);
    }

    /**
     * WPILib uses a North-West-Up coordinate system, but the FTC SDK Robot Coordinate System is a
     * East-North-Up system.
     *
     * @param yawPitchRollAngles {@link YawPitchRollAngles} in the FTC SDK Robot Coordinate System.
     * @return {@link YawPitchRollAngles} transformed to be in the WPILib coordinate system.
     */
    public static Rotation3d SDKRotationToWPILib(YawPitchRollAngles yawPitchRollAngles) {
        return new Rotation3d(
                -yawPitchRollAngles.getRoll(AngleUnit.RADIANS),
                yawPitchRollAngles.getPitch(AngleUnit.RADIANS),
                yawPitchRollAngles.getYaw(AngleUnit.RADIANS));
    }

    public static Pose2D pedroToFTC(Pose pose) {
        // Transform first (in inches)
        Pose ftcPoseInches = new Pose(pose.getX(), pose.getY(), pose.getHeading(),
                PedroCoordinates.INSTANCE)
                .getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);

        // Then convert to meters
        return new Pose2D(DistanceUnit.METER,
                Units.inchesToMeters(ftcPoseInches.getX()),
                Units.inchesToMeters(ftcPoseInches.getY()),
                AngleUnit.RADIANS,
                ftcPoseInches.getHeading());
    }

    public static Pose3d fieldPoseToWPILib(Pose3D pose) {
        return new Pose3d(
                ftcFieldCoordinatesToWPILib(pose.getPosition()), SDKRotationToWPILib(pose.getOrientation()));
    }

    public static Pose3D WPILibToFieldPose(Pose3d pose) {
        return new Pose3D(
                WPILibToFTCFieldCoordinates(pose.getTranslation()), WPILibToSDKRotation(pose.getRotation()));
    }

    public static Pose2d pedroToOrion(Pose pose) {
        double x = Units.inchesToMeters(pose.getX());
        double y = Units.inchesToMeters(pose.getY());

        double heading = pose.getHeading();

        // normalize to [0, 2π)
        heading = heading % (2 * Math.PI);
        if (heading < 0) {
            heading += 2 * Math.PI;
        }

        return new Pose2d(x, y, Rotation2d.fromRadians(heading));
    }



    public static Pose2d ftcToOrion(Pose2D pose2D) {
        return pedroToOrion(ftcToPedro(pose2D));
    }

    public static Pose2d WPILibToOrion(Pose3d pose3d) {
        Pose3D ftcPose = WPILibToFieldPose(pose3d);
        Pose2D ftcPose2D = new Pose2D(
                DistanceUnit.METER,
                ftcPose.getPosition().x,
                ftcPose.getPosition().y,
                AngleUnit.RADIANS,
                ftcPose.getOrientation().getYaw(AngleUnit.RADIANS)
        );

        return ftcToOrion(ftcPose2D);
    }

    public static Pose2D orionToFTC(Pose2d orionPose) {
        return pedroToFTC(orionToPedro(orionPose));
    }

    public static Pose orionToPedro(Pose2d pose2d) {
        double xInches = Units.metersToInches(pose2d.getX());
        double yInches = Units.metersToInches(pose2d.getY());

        return new Pose(xInches, yInches, pose2d.getRotation().getRadians(), PedroCoordinates.INSTANCE);
    }

    public static Pose ftcToPedro(Pose2D llPose) {
        double x = llPose.getX(DistanceUnit.INCH) + 72;
        double y = llPose.getY(DistanceUnit.INCH) + 72;

        double heading = llPose.getHeading(AngleUnit.RADIANS);

        double normalizedHeading = heading > 2 * Math.PI ? heading - 2 * Math.PI : heading;

        return new Pose(y,144 - x,normalizedHeading, PedroCoordinates.INSTANCE);
    }
}