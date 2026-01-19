package org.firstinspires.ftc.lib.orion.odometry;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Odometry {
    private Pose pose;
    private Pose lastPose;
    private double lastTime;

    public Odometry() {}

    public Pose2d getPoseOrion() {
        return CoordinateSystemConverter.pedroToOrion(pose);
    }

    public Pose2D getPoseFTCStandard() {
        return CoordinateSystemConverter.pedroToFTC(pose);
    }

    public Pose2d getPoseWPILib() {
        Pose2D ftcStandard = getPoseFTCStandard();

        Position pos = new Position(DistanceUnit.METER, ftcStandard.getX(DistanceUnit.METER), ftcStandard.getY(DistanceUnit.METER), 0.0, System.nanoTime());
        YawPitchRollAngles ang = new YawPitchRollAngles(AngleUnit.RADIANS, ftcStandard.getHeading(AngleUnit.RADIANS), 0, 0, System.nanoTime());

        return CoordinateSystemConverter.fieldPoseToWPILib(new Pose3D(pos, ang)).toPose2d();
    }

    public void resetPose() {
        pose = new Pose();
    }

    public void resetPose(Pose pose) {
        this.pose = pose;
    }

    public void resetPose(Pose2d pose2d) {
        Translation3d translation = new Translation3d(pose2d.getTranslation());
        Position pos = CoordinateSystemConverter.WPILibToFTCFieldCoordinates(translation);
        pose = new Pose(pos.x, pos.y, pose2d.getRotation().getRadians());
    }

    public void resetPose(Pose2D pose2D) {
        pose = new Pose(pose2D.getX(DistanceUnit.METER), pose2D.getY(DistanceUnit.METER), pose2D.getHeading(AngleUnit.RADIANS));
    }
    /**
     * Gets the x velocity of the Robot
     * @return the x velocity of the robot (m/s)
     */

    public double getXVelocity() {
        double time = System.nanoTime();

        double dt = time - lastTime;
        double dx = pose.getX() - lastPose.getX();

        double dxMeters = Units.inchesToMeters(dx);

        return dxMeters / dt;
    }

    /**
     * Gets the y velocity of the Robot
     * @return the y velocity of the robot (m/s)
     */
    public double getYVelocity() {
        double time = System.nanoTime();

        double dt = time - lastTime;
        double dy = pose.getY() - lastPose.getY();

        double dyMeters = Units.inchesToMeters(dy);

        return dyMeters/dt;
    }

    /**
     * Gets the rotational velocity of the robot
     * @return the rotation velocity of the robot in rad/s
     */
    public double getRotationalVelocity() {
        double time = System.nanoTime();

        double dt = time - lastTime;
        double dtheta = pose.getHeading() - lastPose.getHeading();

        return dtheta/dt;
    }

    public double getVelocityMagnitude() {
        return Math.hypot(getXVelocity(), getYVelocity());
    }

    /**
     * Updates the odometry with the latest pose from the Follower.
     * Should be called every loop after Follower.update
     * @param pose the latest pose from the Follower
     */
    public void update(Pose pose) {
        // Prevent large jumps in heading (most likely from a bad gyro reading)
        if (Math.abs(pose.getHeading() - lastPose.getHeading()) > Units.degreesToRadians(40)) {
            return;
        }

        this.lastPose = this.pose;
        this.pose = pose;

        this.lastTime = System.nanoTime();
    }
}
