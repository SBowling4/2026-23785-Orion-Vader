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
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotCalculator;

public class VisionSubsystem {
    private Limelight3A limelight;
    private LLResult result;

    private double lastTimeReset = 0;
    private double lastTimeHeadingReset = 0;
    private double lastHeadingResetVal = 0;

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;

    private Pose2D llPoseMT1;
    private Pose2D llPoseMT2;
    private DriveSubsystem driveSubsystem;

    private static VisionSubsystem instance;

    private VisionSubsystem(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);

        driveSubsystem = DriveSubsystem.getInstance();

        lastTimeReset = 0;
        lastTimeHeadingReset = 0;
        lastHeadingResetVal = 0;
    }

    public void start() {
        if (Robot.alliance == Alliance.UNKNOWN) {
            throw new IllegalStateException("Alliance not set");
        }

        limelight.start();
    }

    public void loop() {
        if (limelight == null) return;

        limelight.updateRobotOrientation(driveSubsystem.getFollowerPoseFTC().getHeading(AngleUnit.DEGREES));

        result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            llPoseMT1 = null;
            return;
        }

        llPoseMT1 = PoseObjectConverter.pose3DToPose2D(result.getBotpose());
        llPoseMT2 = PoseObjectConverter.pose3DToPose2D(result.getBotpose_MT2());

        if (gamepad1.right_bumper) return;

        if (shouldUpdateHeading(llPoseMT1)) {
            lastTimeHeadingReset = BaseOpMode.getOpModeTimeSeconds();
            lastHeadingResetVal = CoordinateSystemConverter.ftcToPedro(llPoseMT1).getHeading();

            driveSubsystem.resetPoseHeading(
                    lastHeadingResetVal
            );

            return;
        }


        if (shouldUpdate(llPoseMT2)) {
            lastTimeReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPoseNoHeading(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT2)
            );
        }
    }

    public void autoLoop(boolean isShooting) {
        if (limelight == null) return;

        result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            llPoseMT1 = null;
            return;
        }

        if (ShotCalculator.getInstance().getShootingParameters().isFar()) return;

        llPoseMT1 = PoseObjectConverter.pose3DToPose2D(result.getBotpose());

        if (shouldUpdate(llPoseMT1)) {
            lastTimeHeadingReset = BaseOpMode.getOpModeTimeSeconds();

            driveSubsystem.resetPose(
                    CoordinateSystemConverter.ftcToPedro(llPoseMT1),
                    true
            );
        }
    }

    private boolean shouldUpdate(Pose2D llPose) {
        return  (!Robot.tuningMode &&
                poseValid(llPose) &&
                driveSubsystem.getAngularVelocity() < Math.PI / 2);
    }

    private boolean shouldUpdateHeading(Pose2D llPose) {
        return (!Robot.tuningMode &&
                poseValid(llPose) &&
                driveSubsystem.getAngularVelocity() < Math.PI / 2 &&
                BaseOpMode.getOpModeTimeSeconds() - lastTimeHeadingReset > 1) ||
                Math.abs(getDiff().getHeading(AngleUnit.DEGREES)) > 30;
    }

    private Pose2D getDiff() {
        if (llPoseMT1 == null || llPoseMT2 == null) return new Pose2D(DistanceUnit.METER, 0 , 0, AngleUnit.RADIANS, 0);

        return new Pose2D(
                DistanceUnit.METER,
                llPoseMT1.getX(DistanceUnit.METER) - llPoseMT2.getX(DistanceUnit.METER),
                llPoseMT1.getY(DistanceUnit.METER) - llPoseMT2.getY(DistanceUnit.METER),
                AngleUnit.RADIANS,
                llPoseMT1.getHeading(AngleUnit.RADIANS) - llPoseMT2.getHeading(AngleUnit.RADIANS)
                );
    }

    private boolean poseValid(Pose2D llPose) {
        if (llPose == null) return false;
        return Math.abs(llPose.getX(DistanceUnit.METER)) < (Field.FIELD_LENGTH / 2.0) || Math.abs(llPose.getY(DistanceUnit.METER)) < (Field.FIELD_LENGTH / 2.0);
    }

    private double isNan(double num) {
        if (Double.isNaN(num)) {
            return -1;
        }

        return num;
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Vision/Pose Valid", poseValid(llPoseMT1));

        packet.put("Vision/Heading Reset/Time", lastTimeHeadingReset);
        packet.put("Vision/Heading Reset/Should Reset", shouldUpdateHeading(llPoseMT1));
        packet.put("Vision/Heading Reset/Last Reset Val", lastHeadingResetVal);

        packet.put("Vision/Pose Reset/Time", lastTimeReset);
        packet.put("Vision/Should Update", shouldUpdate(llPoseMT2));



        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            return;
        }

        packet.put("Vision/Staleness", result.getStaleness());
        packet.put("Vision/STD Deviation MT1/x", isNan(result.getStddevMt1()[0]));
        packet.put("Vision/STD Deviation MT1/y", isNan(result.getStddevMt1()[1]));
        packet.put("Vision/STD Deviation MT1/heading", isNan(result.getStddevMt1()[5]));
        packet.put("Vision/STD Deviation MT2/x", isNan(result.getStddevMt2()[0]));
        packet.put("Vision/STD Deviation MT2/y", isNan(result.getStddevMt2()[1]));
        packet.put("Vision/STD Deviation MT2/heading", isNan(result.getStddevMt2()[5]));

        packet.put("Vision/Pose Diff/x", getDiff().getX(DistanceUnit.METER));
        packet.put("Vision/Pose Diff/y", getDiff().getY(DistanceUnit.METER));
        packet.put("Vision/Pose Diff/heading", getDiff().getHeading(AngleUnit.DEGREES));

        packet.put("Vision/PoseMT1/Pose x",
                llPoseMT1 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) :  llPoseMT1.getX(DistanceUnit.INCH));

        packet.put("Vision/PoseMT1/Pose y",
                llPoseMT1 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) : llPoseMT1.getY(DistanceUnit.INCH));

        packet.put("Vision/PoseMT1/Pose heading",
                llPoseMT1 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) : llPoseMT1.getHeading(AngleUnit.RADIANS));

        packet.put("Vision/PoseMT2/Pose x",
                llPoseMT2 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) : llPoseMT2.getX(DistanceUnit.INCH));

        packet.put("Vision/PoseMT2/Pose y",
                llPoseMT2 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) : llPoseMT2.getY(DistanceUnit.INCH));

        packet.put("Vision/PoseMT2/Pose heading",
                llPoseMT2 == null ? new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS, 0) : llPoseMT2.getHeading(AngleUnit.RADIANS));

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
