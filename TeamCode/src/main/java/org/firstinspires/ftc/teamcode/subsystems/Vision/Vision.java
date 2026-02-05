package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.orion.util.converters.CoordinateSystemConverter;
import org.firstinspires.ftc.lib.orion.util.converters.PoseObjectConverter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;

public class Vision {
    private Limelight3A limelight;
    private LLResult result;

    private double lastTimeReset = 0;
    private double lastTimeFullReset = 0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Pose2D llPoseMT1;
    private Pose2D llPoseMT2;

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
            llPoseMT1 = null;
            llPoseMT2 = null;
            return;
        }

        double yaw = driveSubsystem.getFollowerPoseFTC().getHeading(AngleUnit.DEGREES);

        yaw = yaw < 0 ? yaw + 360 : yaw;

        limelight.updateRobotOrientation(yaw);

        llPoseMT1 = PoseObjectConverter.pose3DToPose2D(result.getBotpose());
        llPoseMT2 = PoseObjectConverter.pose3DToPose2D(result.getBotpose_MT2());


        if (shouldForce()) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();
            lastTimeFullReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPose(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT1),
                    true
            );

            return;
        }

        if (shouldFullUpdate()) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();
            lastTimeFullReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPose(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT1),
                    true
            );

            return;
        }

        if (shouldUpdate()) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPose(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT2),
                    false
            );

            return;
        }
    }

    private boolean shouldUpdate() {
        return !driveSubsystem.isMoving() && BaseOpMode.getOpModeTimeSeconds() - lastTimeReset < 2 && !gamepad1.right_bumper;
    }

    private boolean shouldForce() {
        return gamepad1.x;
    }

    private boolean shouldFullUpdate() {
        return BaseOpMode.getOpModeTimeSeconds() - lastTimeFullReset > 10 && !driveSubsystem.isMoving() && !gamepad1.right_bumper;
    }



    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Vision/Last ReLoc Time", lastTimeReset);

        if (llPoseMT1 == null) {
            packet.put("Vision/PoseMT1/Pose x", 0);
            packet.put("Vision/PoseMT1/Pose y", 0);
            packet.put("Vision/PoseMT1/Pose heading", 0);

            packet.put("Vision/PoseMT2/Pose x", 0);
            packet.put("Vision/PoseMT2/Pose y", 0);
            packet.put("Vision/PoseMT2/Pose heading", 0);


            packet.put("Vision/PedroPose/Pose x",
                    0);
            packet.put("Vision/PedroPose/Pose y",
                    0);
            packet.put("Vision/PedroPose/Pose heading",
                    0);
            return;
        }
        packet.put("Vision/PoseMT1/Pose x",
                llPoseMT1.getX(DistanceUnit.INCH));

        packet.put("Vision/PoseMT1/Pose y",
                llPoseMT1.getX(DistanceUnit.INCH));

        packet.put("Vision/PoseMT1/Pose heading",
                llPoseMT1.getHeading(AngleUnit.RADIANS));


        packet.put("Vision/PoseMT2/Pose x",
                llPoseMT2.getX(DistanceUnit.INCH));

        packet.put("Vision/PoseMT2/Pose y",
                llPoseMT2.getY(DistanceUnit.INCH));

        packet.put("Vision/PoseMT2/Pose heading",
                llPoseMT2.getHeading(AngleUnit.RADIANS));

        Pose pedroPose = CoordinateSystemConverter.ftcToPedro(llPoseMT1);

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
