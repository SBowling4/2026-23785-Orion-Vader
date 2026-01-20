package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.PoseConverter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.orion.util.converters.PoseObjectConverter;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Pose2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Rotation2d;
import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation3d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;
    private LLResult result;

    private final HardwareMap hardwareMap;

    private Pose2D llPose;
    private double lastUpdateTime = 0;

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
            llPose = null;
            return;
        }

        llPose = PoseObjectConverter.pose3DToPose2D(result.getBotpose_MT2());

        if ((BaseOpMode.getOpModeTimeSeconds() - lastUpdateTime) > 20 && driveSubsystem.isMoving()) {
            driveSubsystem.resetPose(CoordinateSystemConverter.limelightToPedro(llPose));
            lastUpdateTime = BaseOpMode.getOpModeTimeSeconds();

            return;
        }

        Pose3D llPose3D = PoseObjectConverter.pose2DToPose3D(llPose);

        Translation3d translation = CoordinateSystemConverter.ftcFieldCoordinatesToWPILib(llPose3D.getPosition());

        Pose2d wpiPose = new Pose2d(translation.toTranslation2d(), Rotation2d.fromDegrees(llPose3D.getOrientation().getYaw()));

        driveSubsystem.addVisionMeasurement(
                wpiPose,
                result.getTimestamp()
//                VecBuilder.fill(
//                        wpiPose.getX() * .5, //TODO: Figure out how to do this
//                        wpiPose.getX() * .5,
//                        wpiPose.getX() * .5
//                )
        );
    }



    public void setTelemetry(TelemetryPacket packet) {
        if (llPose == null) {
            packet.put("Vision/Pose/Pose x", 0);
            packet.put("Vision/Pose/Pose y", 0);
            packet.put("Vision/Pose/Pose heading", 0);

            return;
        }

        packet.put("Vision/Pose/Pose x", Units.metersToInches(llPose.getX(DistanceUnit.METER)));
        packet.put("Vision/Pose/Pose y", Units.metersToInches(llPose.getY(DistanceUnit.METER)));
        packet.put("Vision/Pose/Pose heading",Units.degreesToRadians(llPose.getHeading(AngleUnit.DEGREES)));
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
