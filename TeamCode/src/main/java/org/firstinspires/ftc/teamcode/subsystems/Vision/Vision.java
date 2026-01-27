package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private double lastYawDeg = 0.0;
    private boolean didReset = false;

    private final HardwareMap hardwareMap;

    private Pose2D llPose;

    private DriveSubsystem driveSubsystem;
    private OrionIMU imu;

    private static Vision instance;

    private Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(0);

        if (Robot.alliance == Alliance.UNKNOWN) {
            throw new IllegalStateException("Alliance not set");
        }

        driveSubsystem = DriveSubsystem.getInstance();

        imu = new OrionIMU(
                hardwareMap,
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
    }

    public void start() {
        limelight.start();
    }

    public void loop() {
        if (limelight == null) return;

        double yawDeg = imu.getAbsoluteYaw(AngleUnit.DEGREES);

        if (compensateForJump(yawDeg)) return;

        limelight.updateRobotOrientation(yawDeg);

        result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            llPose = null;
            return;
        }

        llPose = PoseObjectConverter.pose3DToPose2D(result.getBotpose());

        if (!driveSubsystem.isMoving()) {
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

        lastYawDeg = yawDeg;
        didReset = false;
    }

    /**
     * Detects and compensates for sudden IMU yaw jumps.
     */
    private boolean compensateForJump(double currentYawDeg) {
        double diff = shortestAngleDiffDeg(currentYawDeg, lastYawDeg);

        if (Math.abs(diff) > 60 && !didReset) {
            imu.resetYaw(Math.toRadians(lastYawDeg));
            didReset = true;
            return true;
        }

        return false;
    }

    /**
     * Returns shortest signed angular difference in degrees.
     */
    private double shortestAngleDiffDeg(double a, double b) {
        double diff = (a - b + 180) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    /**
     * @param val the value to reset the imu to - radians
     */
    public void resetImu(double val) {
        imu.resetYaw(val);
    }

    public void setTelemetry(TelemetryPacket packet) {
        packet.put("Vision/IMU/Yaw", imu.getYaw(AngleUnit.DEGREES));
        packet.put("Vision/IMU/Absolute Yaw", imu.getAbsoluteYaw(AngleUnit.DEGREES));

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

    public static Vision getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Vision(hardwareMap);
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
