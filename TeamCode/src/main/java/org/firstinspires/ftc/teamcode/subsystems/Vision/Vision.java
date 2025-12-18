package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;
    private LLResultTypes.FiducialResult goodTag;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public boolean llValid = true;

    private Pose3D visPose;

    private static Vision instance;

    /**
     * Private constructor for singleton pattern.
     *
     * @param hardwareMap Hardware map from the robot.
     */
    private Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
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
            llValid = false;
            return;
        }

        llValid = true;

        LLResult result = limelight.getLatestResult();

        if (result == null) return;

        setVisPose(result.getBotpose());

        setTelemetry();
    }


    /**
     * * Get the current vision pose.
     *
     * @return The current vision pose.
     */
    public Pose3D getVisPose() {
        return visPose;
    }

    /**
     * Set the current vision pose.
     *
     * @param pose The new vision pose.
     */
    public void setVisPose(Pose3D pose) {
        this.visPose = pose;
    }

    private void setTelemetry() {
        telemetry.addLine("//Vision//");
        telemetry.addData("Limelight Valid", llValid);
        telemetry.addLine();
    }


    /**
     * Get the singleton instance of the Vision subsystem.
     *
     * @param hardwareMap The hardware map to use for initialization.
     * @return The singleton instance of the Vision subsystem.
     */
    public static Vision getInstance(HardwareMap hardwareMap, Telemetry telemetry) {
        if (instance == null) {
            instance = new Vision(hardwareMap, telemetry);
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
