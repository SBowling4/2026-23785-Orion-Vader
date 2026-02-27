package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

public class DriveSubsystem {

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public Follower follower;

    private final Pose resetPose = new Pose(9, 9, Math.toRadians(180));

    private VisionSubsystem visionSubsystem;

    private static DriveSubsystem instance;

    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Robot.lastPose);
    }

    public void autoInit(Follower follower) {
        this.follower = follower;
    }


    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        // Get driver inputs
        double inputX = -gamepad1.left_stick_y;
        double inputY = -gamepad1.left_stick_x;
        double inputHeading = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(
                inputX,
                inputY,
                inputHeading,
                false,
                Robot.alliance == Alliance.BLUE ? Math.toRadians(180) : Math.toRadians(0)
        );

        // Reset the driver's forward direction (doesn't affect absolute heading)
        if (gamepad1.share) {
            resetPoseHeading();
        }

        // Full pose reset (resets both absolute and driver heading)
        if (gamepad1.left_stick_button) {
            resetPose();

            update();
            return;
        }

       update();
    }

    /**
     * Updates the follower, odometry, and pose estimator
     */
    private void update() {
        follower.update();
    }

    /**
     * Resets the follower, odometry, and pose estimator to the initial reset pose
     */
    public void resetPose() {
        follower.setPose(resetPose);
    }

    /**
     * Resets the follower, odometry, and pose estimator to the given pose
     * Primarily used for ll vision resets
     * @param pose the pose to reset to
     */
    public void resetPose(Pose pose, boolean heading) {
        if (heading) {
            follower.setPose(pose);
        } else {
            follower.setPose(new Pose(pose.getX(), pose.getY(), getFollowerPose().getHeading(), PedroCoordinates.INSTANCE));
        }
    }

    public void resetPoseNoHeading(Pose pose) {
        follower.setPose(new Pose(pose.getX(), pose.getY(), follower.getPose().getHeading(), PedroCoordinates.INSTANCE));
    }

    /**
     * Resets only the heading of the pose to the alliance standard
     * Keeps the x and y coordinates the same
     */
    public void resetPoseHeading() {
        double heading = Robot.alliance == Alliance.BLUE ? Math.toRadians(180) : Math.toRadians(0);
        Pose headingResetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), heading);

        follower.setPose(headingResetPose);
    }

    public void resetPoseHeading(double heading) {
        Pose headingResetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), heading);

        follower.setPose(headingResetPose);
    }

    /**
     * Returns the absolute field-relative pose from Pedro Pathing
     * This is what should be used for odometry and localization
     * Coordinates: Pedro Pathing Standard
     */
    public Pose getFollowerPose() {
        return follower.getPose();
    }

    /**
     * Gets the odometry pose in FTC standard coordinates
     * Coordinates: FTC Standard
     */
    public Pose2D getFollowerPoseFTC() {
        return CoordinateSystemConverter.pedroToFTC(getFollowerPose());
    }

    /**
     * Gets the estimated pose from the pose estimator (with vision fusion)
     * Coordinates: Orion
     */
    public Pose2d getFollowerPoseOrion() {
        return CoordinateSystemConverter.pedroToOrion(getFollowerPose());
    }


    /**
     * @return true if the robot is currently moving
     */
    public boolean isMoving() {
        double mag = follower.getVelocity().getMagnitude();
        double rot = follower.getAngularVelocity();

        return mag > .2 || Math.abs(rot) > .2;
    }

    public double getXVelocity() {
        return Units.inchesToMeters(follower.getVelocity().getXComponent());
    }

    public double getYVelocity() {
        return Units.inchesToMeters(follower.getVelocity().getYComponent());
    }

    public double getVelocityMagnitude() {
        return Units.inchesToMeters(follower.getVelocity().getMagnitude());
    }

    public double getAngularVelocity() {
        return follower.getAngularVelocity();
    }

    /**
     * @return the distance from the robot to the goal in meters
     */
    public double getDistanceToGoal() {
        if (Robot.alliance == Alliance.BLUE) {
            return getFollowerPoseOrion().getTranslation().getDistance(Field.BLUE_GOAL);
        } else {
            return getFollowerPoseOrion().getTranslation().getDistance(Field.RED_GOAL);
        }
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Drive/Follower Pose/Pose x", getFollowerPoseFTC().getX(DistanceUnit.INCH));
        packet.put("Drive/Follower Pose/Pose y", getFollowerPoseFTC().getY(DistanceUnit.INCH));
        packet.put("Drive/Follower Pose/Pose heading", getFollowerPoseFTC().getHeading(AngleUnit.RADIANS));

        packet.put("Drive/Pose Orion/X", getFollowerPoseOrion().getX());
        packet.put("Drive/Pose Orion/Y", getFollowerPoseOrion().getY());
        packet.put("Drive/Pose Orion/Heading", getFollowerPoseOrion().getRotation().getRadians());

        packet.put("Drive/Velocity/X", getXVelocity());
        packet.put("Drive/Velocity/Y", getYVelocity());
        packet.put("Drive/Velocity/Angular", getAngularVelocity());
        packet.put("Drive/Velocity/Magnitude", getVelocityMagnitude());
        packet.put("Drive/Velocity/Moving", isMoving());


        packet.put("Drive/Distance to Goal", getDistanceToGoal());
    }

    public static DriveSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new DriveSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("DriveSubsystem not initialized. Call getInstance(telemetry, hardwareMap, gamepad1) first.");
        }
        return instance;
    }
}