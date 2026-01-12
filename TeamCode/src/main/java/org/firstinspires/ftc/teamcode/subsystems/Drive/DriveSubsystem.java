package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.PoseConverter;
import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.orion.odometry.Odometry;
import org.firstinspires.ftc.lib.orion.odometry.PoseEstimator;
import org.firstinspires.ftc.lib.pedroPathing.Constants;
import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.wpilib.math.Matrix;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N1;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N3;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.lib.orion.util.Alliance;

public class DriveSubsystem {

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public Follower follower;

    public Odometry odometry;
    public PoseEstimator poseEstimator;

    private double driverHeadingOffset = 0;

    private static DriveSubsystem instance;

    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        odometry = new Odometry();

        poseEstimator = new PoseEstimator(
                odometry,
                VecBuilder.fill(0.4, 0.4, 0.4),
                VecBuilder.fill(0.3, 0.3, 0.3)
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Robot.lastPose);
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        // Get driver inputs
        double inputX = -gamepad1.left_stick_y;
        double inputY = -gamepad1.left_stick_x;
        double inputHeading = -gamepad1.right_stick_x;

        // Rotate driver inputs by the heading offset to maintain field-oriented control
        // relative to the driver's chosen forward direction
        double cos = Math.cos(-driverHeadingOffset);
        double sin = Math.sin(-driverHeadingOffset);
        double rotatedX = inputX * cos - inputY * sin;
        double rotatedY = inputX * sin + inputY * cos;

        follower.setTeleOpDrive(
                -rotatedX,
                -rotatedY,
                inputHeading,
                false
        );

        // Reset the driver's forward direction (doesn't affect absolute heading)
        if (gamepad1.share) {
            resetDriverHeading();
        }

        // Full pose reset (resets both absolute and driver heading)
        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            follower.setPose(new Pose(72, 72, 0));
            driverHeadingOffset = 0;
        }

        // Update follower first
        follower.update();

        // Update odometry with the absolute field-relative pose
        odometry.update(follower.getPose());

        // Update pose estimator
        poseEstimator.update();
    }

    /**
     * Returns the absolute field-relative pose from Pedro Pathing
     * This is what should be used for odometry and localization
     */
    public Pose getFollowerPose() {
        return follower.getPose();
    }

    /**
     * Returns the pose adjusted to field coordinates (origin at field center)
     */
    public Pose getFollowerPoseFieldAdj() {
        return new Pose(
                getFollowerPose().getX() - 72,
                getFollowerPose().getY() - 72,
                getFollowerPose().getHeading()
        );
    }

    /**
     * Returns the pose relative to the driver's chosen forward direction
     * This is useful for driver-station display but should NOT be used for odometry
     */
    public Pose getDriverRelativePose() {
        return new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getHeading() - driverHeadingOffset
        );
    }

    /**
     * Gets the estimated pose from the pose estimator (with vision fusion)
     */
    public Pose2D getEstimatedPoseFTC() {
        Pose2d poseEstimatorPose = poseEstimator.getEstimatedPosition();
        Pose3d pose3d = new Pose3d(
                poseEstimatorPose.getX(),
                poseEstimatorPose.getY(),
                0,
                new Rotation3d(poseEstimatorPose.getRotation())
        );
        Pose3D ftcPose = CoordinateSystems.WPILibToFieldPose(pose3d);

        return new Pose2D(
                DistanceUnit.METER,
                ftcPose.getPosition().x,
                ftcPose.getPosition().y,
                AngleUnit.RADIANS,
                ftcPose.getOrientation().getYaw(AngleUnit.RADIANS)
        );
    }

    public Pose2d getEstimatedPoseWPILIB() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the driver's forward direction to the current robot heading
     * This does NOT affect the absolute field-relative heading used for odometry
     */
    public void resetDriverHeading() {
        driverHeadingOffset = follower.getHeading();
    }

    /**
     * Gets the absolute field-relative heading (for odometry/localization)
     */
    public double getAbsoluteHeading() {
        return follower.getHeading();
    }

    /**
     * Gets the heading relative to the driver's chosen forward direction
     */
    public double getDriverRelativeHeading() {
        return follower.getHeading() - driverHeadingOffset;
    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds, Matrix<N3, N1> visionStdDev) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds, visionStdDev);
    }

    public double getDistanceToGoal() {
        Pose3D estPose = new Pose3D(
                new Position(
                        DistanceUnit.METER,
                        getEstimatedPoseFTC().getX(DistanceUnit.METER),
                        getEstimatedPoseFTC().getY(DistanceUnit.METER),
                        0.0,
                        0
                ),
                new YawPitchRollAngles(
                        AngleUnit.RADIANS,
                        getEstimatedPoseFTC().getHeading(AngleUnit.RADIANS),
                        0,
                        0,
                        0
                )
        );

        Pose2d wpiLibPose = PoseConverter.ftcToWPILib(estPose).toPose2d();

        if (Robot.alliance == Alliance.BLUE) {
            return wpiLibPose.getTranslation().getDistance(Field.BLUE_GOAL);
        } else {
            return wpiLibPose.getTranslation().getDistance(Field.RED_GOAL);
        }
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Drive//");
        telemetry.addData("X", getEstimatedPoseFTC().getX(DistanceUnit.METER));
        telemetry.addData("Y", getEstimatedPoseFTC().getY(DistanceUnit.METER));
        telemetry.addData("Absolute Heading (deg)", Math.toDegrees(getAbsoluteHeading()));
        telemetry.addData("Driver Heading (deg)", Math.toDegrees(getDriverRelativeHeading()));
        telemetry.addData("Distance to Goal", getDistanceToGoal());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Drive/Estimated Pose/Pose x", Units.metersToInches(getEstimatedPoseFTC().getX(DistanceUnit.METER)));
        packet.put("Drive/Estimated Pose/Pose y", Units.metersToInches(getEstimatedPoseFTC().getY(DistanceUnit.METER)));
        packet.put("Drive/Estimated Pose/Pose heading", getEstimatedPoseFTC().getHeading(AngleUnit.RADIANS));

        packet.put("Drive/Odometry Pose/Pose x", odometry.getPoseFTCStandard().getX(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose y", odometry.getPoseFTCStandard().getY(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose heading", odometry.getPoseFTCStandard().getHeading(AngleUnit.RADIANS));

        packet.put("Drive/Absolute Heading (rad)", getAbsoluteHeading());
        packet.put("Drive/Driver Heading (rad)", getDriverRelativeHeading());
        packet.put("Drive/Distance to Goal", getDistanceToGoal());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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