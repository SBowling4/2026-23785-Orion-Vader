package org.firstinspires.ftc.lib.orion.odometry;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Odometry {
    private Pose pose;

    public Odometry() {}

    public Pose2D getPoseFTCStandard() {
        double pedroX = pose.getX();
        double pedroY = pose.getY();

        double ftcY = pedroY - 72;
        double ftcX = pedroX - 72;

        return new Pose2D(DistanceUnit.INCH, ftcX, ftcY, AngleUnit.RADIANS, pose.getHeading());

//        return PoseConverter.poseToPose2D(pose, FTCCoordinates.INSTANCE);
    }

    public Pose2d getPoseWPILib() {
        Pose2D ftcStandard = getPoseFTCStandard();

        Position pos = new Position(DistanceUnit.METER, ftcStandard.getX(DistanceUnit.METER), ftcStandard.getY(DistanceUnit.METER), 0.0, System.nanoTime());
        YawPitchRollAngles ang = new YawPitchRollAngles(AngleUnit.RADIANS, ftcStandard.getHeading(AngleUnit.RADIANS), 0, 0, System.nanoTime());

        return CoordinateSystems.fieldPoseToWPILib(new Pose3D(pos, ang)).toPose2d();
    }

    public void resetPose() {
        pose = new Pose();
    }

    public void resetPose(Pose pose) {
        this.pose = pose;
    }

    public void resetPose(Pose2d pose2d) {
        Translation3d translation = new Translation3d(pose2d.getTranslation());
        Position pos = CoordinateSystems.WPILibToFieldCoordinates(translation);
        pose = new Pose(pos.x, pos.y, pose2d.getRotation().getRadians());
    }

    public void resetPose(Pose2D pose2D) {
        pose = new Pose(pose2D.getX(DistanceUnit.METER), pose2D.getY(DistanceUnit.METER), pose2D.getHeading(AngleUnit.RADIANS));
    }


    /**
     * Updates the odometry with the latest pose from the Follower.
     * Should be called every loop after Follower.update
     * @param pose the latest pose from the Follower
     */
    public void update(Pose pose) {
        this.pose = pose;
    }
}
