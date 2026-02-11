package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.orion.util.Field;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.orion.util.converters.PoseObjectConverter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

public class VisionSubsystem {
    private Limelight3A limelight;

    private double lastTimeReset = 0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Pose2D llPoseMT1;
    private DriveSubsystem driveSubsystem;

    private static VisionSubsystem instance;

    private VisionSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);

        if (Robot.alliance == Alliance.UNKNOWN) {
            throw new IllegalStateException("Alliance not set");
        }

        driveSubsystem = DriveSubsystem.getInstance();

        lastTimeReset = 0;
    }

    public void start() {
        limelight.start();
    }

    public void loop() {
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            llPoseMT1 = null;
            return;
        }

        llPoseMT1 = PoseObjectConverter.pose3DToPose2D(result.getBotpose());

        if (shouldUpdate(llPoseMT1)) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPose(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT1),
                    true
            );
        }
    }

    private boolean shouldUpdate(Pose2D llPose) {
        return  (!gamepad1.right_bumper || gamepad1.x) && !Robot.tuningMode && poseValid(llPose);
    }

    private boolean poseValid(Pose2D llPose) {
        if (llPose == null) return false;
        return Math.abs(llPose.getX(DistanceUnit.METER)) < (Field.FIELD_LENGTH / 2.0) || Math.abs(llPose.getY(DistanceUnit.METER)) < (Field.FIELD_LENGTH / 2.0);
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Vision/Last Reset Time", lastTimeReset);
        packet.put("Vision/Should Update", shouldUpdate(llPoseMT1));
        packet.put("Vision/Pose Valid", poseValid(llPoseMT1));


        if (llPoseMT1 == null) {
            packet.put("Vision/Pose/Pose x", 0);
            packet.put("Vision/Pose/Pose y", 0);
            packet.put("Vision/Pose/Pose heading", 0);

            return;
        }
        packet.put("Vision/Pose/Pose x",
                llPoseMT1.getX(DistanceUnit.INCH));

        packet.put("Vision/Pose/Pose y",
                llPoseMT1.getY(DistanceUnit.INCH));

        packet.put("Vision/Pose/Pose heading",
                llPoseMT1.getHeading(AngleUnit.RADIANS));

    }

    public static VisionSubsystem getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new VisionSubsystem(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException(
                    "Vision not initialized. Call getInstance(hardwareMap) first."
            );
        }
        return instance;
    }
}
