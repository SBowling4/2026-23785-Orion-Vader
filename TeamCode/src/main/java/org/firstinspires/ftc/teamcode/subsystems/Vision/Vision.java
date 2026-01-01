package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.trobotix.CoordinateSystems;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;

    private final HardwareMap hardwareMap;

    private Optional<Pose3D> botPose;

    DriveSubsystem driveSubsystem;

    private static Vision instance;

    /**
     * Private constructor for singleton pattern.
     *
     * @param hardwareMap Hardware map from the robot.
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

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            botPose = Optional.empty();
            return;
        }

        botPose = Optional.of(result.getBotpose());

        Translation3d translation = CoordinateSystems.fieldCoordinatesToWPILib(botPose.get().getPosition());

        Pose2d pose = new Pose2d(translation.toTranslation2d(), Rotation2d.fromRadians(botPose.get().getOrientation().getYaw(AngleUnit.RADIANS)));

        driveSubsystem.addVisionMeasurement(pose, result.getTimestamp());
    }




    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Vision//");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());
        telemetry.addLine("------------");

        if (botPose.isEmpty()) {
            telemetry.addLine("Pose Invalid");
            telemetry.addLine();
            return;
        }

        telemetry.addData("X", botPose.get().getPosition().x);
        telemetry.addData("Y", botPose.get().getPosition().y);
        telemetry.addData("Yaw", botPose.get().getOrientation().getYaw());
        telemetry.addLine();
    }


    /**
     * Get the singleton instance of the Vision subsystem.
     *
     * @param hardwareMap The hardware map to use for initialization.
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
