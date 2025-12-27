package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;

    private final HardwareMap hardwareMap;

    private Pose3D visPose;

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

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) return;

        setVisPose(result.getBotpose());
    }


    /**
     * Get the current vision pose.
     *
     * @return The current vision pose.
     */
    public Optional<Pose> getVisPose() {
        if (visPose == null) {
            return Optional.empty();
        }

        Pose2D visPose2d = new Pose2D(DistanceUnit.METER, visPose.getPosition().x, visPose.getPosition().y, AngleUnit.DEGREES, visPose.getOrientation().getYaw());

        Pose ftcStandard = PoseConverter.pose2DToPose(visPose2d, FTCCoordinates.INSTANCE);

        Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        return Optional.of(new Pose(pedroPose.getX() + 72, pedroPose.getY() + 72, pedroPose.getHeading(), PedroCoordinates.INSTANCE));

    }


    /**
     * Set the current vision pose.
     *
     * @param pose The new vision pose.
     */
    public void setVisPose(Pose3D pose) {
        this.visPose = pose;
    }

    public void setTelemetry(Telemetry telemetry) {
        telemetry.addLine("//Vision//");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());
        telemetry.addLine("------------");

        if (getVisPose().isEmpty()) {
            telemetry.addLine("Pose Invalid");
            telemetry.addLine();
            return;
        }

        telemetry.addData("X", getVisPose().get().getX());
        telemetry.addData("Y", getVisPose().get().getY());
        telemetry.addData("Yaw", getVisPose().get().getHeading());
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
