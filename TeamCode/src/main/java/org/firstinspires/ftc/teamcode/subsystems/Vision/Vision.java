package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
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

    private Pose3D llPose;
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

        llPose = result.getBotpose_MT2();

        Translation3d translation = CoordinateSystemConverter.ftcFieldCoordinatesToWPILib(llPose.getPosition());

        Pose2d wpiPose = new Pose2d(translation.toTranslation2d(), Rotation2d.fromDegrees(llPose.getOrientation().getYaw()));

        double x = llPose.getPosition().x;;
        double y = llPose.getPosition().y;;
        double heading = llPose.getOrientation().getYaw();

        Pose2D llPose2d = new Pose2D(
                DistanceUnit.METER,
                x,
                y,
                AngleUnit.DEGREES,
                heading
        );

        Pose2d orionPose = CoordinateSystemConverter.limelightToOrion(llPose2d);

        if ((BaseOpMode.getOpModeTimeSeconds() - lastUpdateTime) > 20 &&  driveSubsystem.getVelocity() < .25) {
            //TODO: Make this code

            lastUpdateTime = BaseOpMode.getOpModeTimeSeconds();
        }

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

    public Optional<Double> getTx() {
        if (!result.isValid()) {
            return Optional.empty();
        }

        return Optional.of(result.getTx());
    }




    public void setTelemetry(TelemetryPacket packet) {
        if (llPose == null) {
            packet.put("Vision/Pose/Pose x", 0);
            packet.put("Vision/Pose/Pose y", 0);
            packet.put("Vision/Pose/Pose heading", 0);

            return;
        }

        packet.put("Vision/Pose/Pose x", Units.metersToInches(llPose.getPosition().x));
        packet.put("Vision/Pose/Pose y", Units.metersToInches(llPose.getPosition().y));
        packet.put("Vision/Pose/Pose heading",Units.degreesToRadians(llPose.getOrientation().getYaw()));
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
