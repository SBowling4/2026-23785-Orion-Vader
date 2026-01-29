package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.hardware.OrionIMU;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.orion.util.converters.PoseObjectConverter;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

public class Vision {
    private Limelight3A limelight;
    private LLResult result;

    private double lastTimeReset = 0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Pose2D llPose;

    private DriveSubsystem driveSubsystem;

    private static Vision instance;

    private Vision(HardwareMap hardwareMap, Gamepad gamepad1) {
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


        result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            llPose = null;
            return;
        }

        llPose = PoseObjectConverter.pose3DToPose2D(result.getBotpose());

        if ((!driveSubsystem.isMoving() && (BaseOpMode.getOpModeTimeSeconds() - lastTimeReset) > 10) || gamepad1.x) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();
            driveSubsystem.resetPoseVis(
                    CoordinateSystemConverter.ftcToPedro(llPose)
            );
        }
//
//        Pose3D llPose3D = PoseObjectConverter.pose2DToPose3D(llPose);
//        Translation3d translation =
//                CoordinateSystemConverter.ftcFieldCoordinatesToWPILib(
//                        llPose3D.getPosition()
//                );
//
//        Pose2d wpiPose = new Pose2d(
//                translation.toTranslation2d(),
//                Rotation2d.fromDegrees(llPose3D.getOrientation().getYaw())
//        );

//        lastYawDeg = yawDeg;
//        didReset = false;
    }




    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Vision/Last ReLoc Time", lastTimeReset);

        if (llPose == null) {
            packet.put("Vision/PoseMT2/Pose x", 0);
            packet.put("Vision/PoseMT2/Pose y", 0);
            packet.put("Vision/PoseMT2/Pose heading", 0);

            packet.put("Vision/PoseMT1/Pose x", 0);
            packet.put("Vision/PoseMT1/Pose y", 0);
            packet.put("Vision/PoseMT1/Pose heading", 0);

            packet.put("Vision/PedroPose/Pose x",
                    0);
            packet.put("Vision/PedroPose/Pose y",
                    0);
            packet.put("Vision/PedroPose/Pose heading",
                    0);
            return;
        }

        packet.put("Vision/PoseMT2/Pose x",
                Units.metersToInches(result.getBotpose_MT2().getPosition().x));
        packet.put("Vision/PoseMT2/Pose y",
                Units.metersToInches(result.getBotpose_MT2().getPosition().y));
        packet.put("Vision/PoseMT2/Pose heading",
                Units.degreesToRadians(result.getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES)));

        packet.put("Vision/PoseMT1/Pose x",
                Units.metersToInches(result.getBotpose().getPosition().x));
        packet.put("Vision/PoseMT1/Pose y",
                Units.metersToInches(result.getBotpose().getPosition().y));
        packet.put("Vision/PoseMT1/Pose heading",
                Units.degreesToRadians(result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES)));

        Pose pedroPose = CoordinateSystemConverter.ftcToPedro(llPose);

        packet.put("Vision/PedroPose/Pose x",
                pedroPose.getX());
        packet.put("Vision/PedroPose/Pose y",
                pedroPose.getY());
        packet.put("Vision/PedroPose/Pose heading",
                pedroPose.getHeading());


    }

    public static Vision getInstance(HardwareMap hardwareMap, Gamepad gamepad1) {
        if (instance == null) {
            instance = new Vision(hardwareMap, gamepad1);
        }
        return instance;
    }

    public static Vision getInstance() {
        if (instance == null) {
            throw new IllegalStateException(
                    "Vision not initialized. Call getInstance(hardwareMap) first."
            );
        }
        return instance;
    }
}
