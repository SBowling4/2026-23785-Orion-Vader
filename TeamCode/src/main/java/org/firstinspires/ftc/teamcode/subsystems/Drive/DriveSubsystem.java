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
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
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

    private Pose pose;

    private double lastHeading = 0;

    private static DriveSubsystem instance;

    private DriveSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        odometry = new Odometry();

        poseEstimator = new PoseEstimator(odometry, VecBuilder.fill(0.6,0.6,0.6), VecBuilder.fill(0.1,0.1,1)); //TODO: fill these in

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72)); //TODO: idk smthn here
    }

    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {

//        mecanum.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );


        if (gamepad1.share) {
            resetHeading();
        }

        follower.update();
        odometry.update(getFollowerPose());
        poseEstimator.update();

    }

    public Pose getFollowerPose() {
        return new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading() + lastHeading);
    }

    public Pose2D getEstimatedPose() {
        Pose2d poseEstimatorPose = poseEstimator.getEstimatedPosition();
        Pose3d pose3d = new Pose3d(poseEstimatorPose.getX(), poseEstimatorPose.getY(), 0, new Rotation3d(poseEstimatorPose.getRotation()));
        Pose3D ftcPose = CoordinateSystems.WPILibToFieldPose(pose3d);

        return new Pose2D(DistanceUnit.METER, ftcPose.getPosition().x, ftcPose.getPosition().y, AngleUnit.RADIANS, ftcPose.getOrientation().getYaw(AngleUnit.RADIANS));
    }

    public void resetHeading() {
        lastHeading += follower.getHeading();

        follower.setPose(follower.getPose().setHeading(0));

    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds);
    }

    public void addVisionMeasurement(Pose2d visPose, double timestampSeconds, Matrix<N3, N1> visionStdDev) {
        poseEstimator.addVisionMeasurement(visPose, timestampSeconds, visionStdDev);
    }

    public double getDistanceToGoal() {
        Pose3D estPose = new Pose3D(new Position(DistanceUnit.METER, getEstimatedPose().getX(DistanceUnit.METER), getEstimatedPose().getY(DistanceUnit.METER), 0.0, 0), new YawPitchRollAngles(AngleUnit.RADIANS, getEstimatedPose().getHeading(AngleUnit.RADIANS), 0, 0, 0));

        Pose2d wpiLibPose = PoseConverter.ftcToWPILib(estPose).toPose2d();

        if (Robot.alliance == Alliance.BLUE) {
            return wpiLibPose.getTranslation().getDistance(Field.BLUE_GOAL);
        } else {
            return wpiLibPose.getTranslation().getDistance(Field.RED_GOAL);
        }
    }



    public void setTelemetry(Telemetry telemetry) {
//        telemetry.addLine("//Drive//");
//        telemetry.addData("X", getEstimatedPose().getX());
//        telemetry.addData("Y", getEstimatedPose().getY());
//        telemetry.addData("Heading", getEstimatedPose().getRotation().getDegrees());
//        telemetry.addData("Distance to Goal", getDistanceToGoal());


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Drive/Estimated Pose/Pose x", Units.metersToInches(getEstimatedPose().getX(DistanceUnit.METER)));
        packet.put("Drive/Estimated Pose/Pose y", Units.metersToInches(getEstimatedPose().getY(DistanceUnit.METER)));
        packet.put("Drive/Estimated Pose/Pose heading", getEstimatedPose().getHeading(AngleUnit.RADIANS));

        packet.put("Drive/Odometry Pose/Pose x", odometry.getPoseFTCStandard().getX(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose y", odometry.getPoseFTCStandard().getY(DistanceUnit.INCH));
        packet.put("Drive/Odometry Pose/Pose heading", odometry.getPoseFTCStandard().getHeading(AngleUnit.RADIANS));
//        packet.put("Drive/PedroPose/Pose x", getFollowerPose().getX());
//        packet.put("Drive/PedroPose/Pose y", getFollowerPose().getY());
//        packet.put("Drive/PedroPose/Pose heading", getFollowerPose().getHeading());



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