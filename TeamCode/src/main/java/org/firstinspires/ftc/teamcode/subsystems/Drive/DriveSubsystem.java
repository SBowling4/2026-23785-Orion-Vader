package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.util.converters.PoseObjectConverter;
import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.orion.odometry.Odometry;
import org.firstinspires.ftc.lib.orion.odometry.PoseEstimator;
import org.firstinspires.ftc.lib.pedroPathing.Constants;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.wpilib.math.Matrix;
import org.firstinspires.ftc.lib.wpilib.math.VecBuilder;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose3d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation3d;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N1;
import org.firstinspires.ftc.lib.wpilib.math.numbers.N3;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class DriveSubsystem {

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    public Follower follower;

    public Odometry odometry;
    public PoseEstimator poseEstimator;

    private double driverHeadingOffset = 0;

    private final Pose resetPose = new Pose(144 - 9.5, 9.5, Math.toRadians(270));

    private PIDFController alignPID;

    private Vision vision;

    private static DriveSubsystem instance;

    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        odometry = new Odometry();

        poseEstimator = new PoseEstimator(
                odometry,
                VecBuilder.fill(0.6, 0.6, .1),
                VecBuilder.fill(0.2, 0.2, 99999)
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Robot.lastPose);

        vision = Vision.getInstance(hardwareMap);

        alignPID = new PIDFController(DriveConstants.align_kP, DriveConstants.align_kI, DriveConstants.align_kD, 0);
    }

    public void autoInit() {
        odometry = new Odometry();

        poseEstimator = new PoseEstimator(
                odometry,
                VecBuilder.fill(0.6, 0.6, .1),
                VecBuilder.fill(0.2, 0.2, 99999)
        );

        vision = Vision.getInstance(hardwareMap);
    }


    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {
        alignPID.setP(DriveConstants.align_kP);
        alignPID.setD(DriveConstants.align_kD);

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

        if (gamepad1.x) {
            align();

            update();
            return;
        }

        // Full pose reset (resets both absolute and driver heading)
        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            resetPose();

            update();
            return;
        }

       update();
    }

    private void update() {
        follower.update();
        odometry.update(follower.getPose());
        poseEstimator.update();
    }

    public void align() {
        if (vision.getTx().isEmpty()) return;

        double pow = alignPID.calculate(vision.getTx().get(), 0);

        follower.setTeleOpDrive(0,0,pow, true);
    }

    public void resetPose() {
        follower.setPose(resetPose);
        driverHeadingOffset = 0;
        odometry.resetPose(resetPose);
        poseEstimator.resetPose(odometry.getPoseWPILib());
    }

    public void resetPoseHeading() {
        double heading = Robot.alliance == Alliance.BLUE ? Math.toRadians(270) : Math.toRadians(90);
        Pose headingResetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), heading);

        follower.setPose(headingResetPose);
        odometry.resetPose(headingResetPose);
        poseEstimator.resetPose(odometry.getPoseWPILib());
    }

    /**
     * Returns the absolute field-relative pose from Pedro Pathing
     * This is what should be used for odometry and localization
     */
    public Pose getFollowerPose() {
        return follower.getPose();
    }

    public Pose2D getOdometryPose() {
        return odometry.getPoseFTCStandard();
    }

    /**
     * Gets the estimated pose from the pose estimator (with vision fusion)
     */
    public Pose2d getEstimatedPose() {
        Pose2d poseEstimatorPose = poseEstimator.getEstimatedPosition();
        Pose3d pose3d = new Pose3d(
                poseEstimatorPose.getX(),
                poseEstimatorPose.getY(),
                0,
                new Rotation3d(poseEstimatorPose.getRotation())
        );

        return CoordinateSystemConverter.WPILibToOrion(pose3d);
    }

    /**
     * Resets the driver's forward direction to the current robot heading
     * This does NOT affect the absolute field-relative heading used for odometry
     */
    public void resetDriverHeading() {
        driverHeadingOffset = follower.getHeading();
        resetPoseHeading();
    }

    public double getVelocity() {
        return odometry.getVelocityMagnitude();
    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds, Matrix<N3, N1> visionStdDev) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds, visionStdDev);
    }

    public double getDistanceToGoal() {
        if (Robot.alliance == Alliance.BLUE) {
            return getEstimatedPose().getTranslation().getDistance(Field.BLUE_GOAL);
        } else {
            return getEstimatedPose().getTranslation().getDistance(Field.RED_GOAL);
        }
    }

    public void setTelemetry(TelemetryPacket packet) {
        Pose2D ftcPose = CoordinateSystemConverter.orionToFTC(getEstimatedPose());

        packet.put("Drive/Estimated Pose/Pose x", ftcPose.getX(DistanceUnit.INCH));
        packet.put("Drive/Estimated Pose/Pose y", ftcPose.getY(DistanceUnit.INCH));
        packet.put("Drive/Estimated Pose/Pose heading", ftcPose.getHeading(AngleUnit.RADIANS));

        packet.put("Drive/Odometry Pose/Pose x", odometry.getPoseFTCStandard().getX(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose y", odometry.getPoseFTCStandard().getY(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose heading", odometry.getPoseFTCStandard().getHeading(AngleUnit.RADIANS));

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