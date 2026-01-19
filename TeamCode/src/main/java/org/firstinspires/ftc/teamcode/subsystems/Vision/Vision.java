package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystems;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;
    private LLResult result;

    private final HardwareMap hardwareMap;

    private Pose3D botPose;

    private DriveSubsystem driveSubsystem;

    private static Vision instance;

    /**
     * Private constructor for singleton pattern.
     */
    private Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initialize the vision system.
     */
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);

        limelight.pipelineSwitch(0);

        if (Robot.alliance == Alliance.UNKNOWN) {
            throw new IllegalStateException("Alliance not set");
        }

        driveSubsystem = DriveSubsystem.getInstance();
    }

    /**
     * Start the vision system.
     */
    public void start() {
        limelight.start();
    }

    /**
     * Main loop for vision processing.
     */
    public void loop() {
        if (limelight == null) {
            return;
        }

        double yaw = driveSubsystem.getOdometryPose().getHeading(AngleUnit.DEGREES);

        limelight.updateRobotOrientation(yaw < 0 ? yaw + 720 : yaw);

        result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            botPose = null;
            return;
        }

        botPose = result.getBotpose_MT2();

        Translation3d translation = CoordinateSystems.fieldCoordinatesToWPILib(botPose.getPosition());

        Pose2d pose = new Pose2d(translation.toTranslation2d(), Rotation2d.fromDegrees(botPose.getOrientation().getYaw()));

//        if (BaseOpMode.getOpModeTimeSeconds() > 20 /*and not moving*/) {
//            //relocalize
//        }

        driveSubsystem.addVisionMeasurement(
                pose,
                result.getTimestamp()
//                VecBuilder.fill(
//                        pose.getX() * .5, //TODO: Figure out how to do this
//                        pose.getX() * .5,
//                        pose.getX() * .5
//                )
        );
    }

    public Optional<Double> getTx() {
        if (!result.isValid()) {
            return Optional.empty();
        }

        return Optional.of(result.getTx());
    }




    public void setTelemetry(TelemetryPacket packet) {
        if (botPose == null) {
            packet.put("Vision/Pose/Pose x", 0);
            packet.put("Vision/Pose/Pose y", 0);
            packet.put("Vision/Pose/Pose heading", 0);

            return;
        }

        packet.put("Vision/Pose/Pose x", Units.metersToInches(botPose.getPosition().x));
        packet.put("Vision/Pose/Pose y", Units.metersToInches(botPose.getPosition().y));
        packet.put("Vision/Pose/Pose heading",Units.degreesToRadians(botPose.getOrientation().getYaw()));
    }

    /**
     * Get the singleton instance of the Vision subsystem.
     *
     * @return The singleton instance of the Vision subsystem.
     */
    public static Vision getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Vision(hardwareMap);
        }
        return instance;
    }


    /**
     * Get the singleton instance of the Vision subsystem.
     *
     * @return The singleton instance of the Vision subsystem.
     */
    public static Vision getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Vision not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }



}
