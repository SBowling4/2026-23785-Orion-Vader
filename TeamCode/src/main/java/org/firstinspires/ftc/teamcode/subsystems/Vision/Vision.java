package org.firstinspires.ftc.teamcode.subsystems.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Optional;

public class Vision {
    private Limelight3A limelight;
    private LLResultTypes.FiducialResult goodTag;

    private final HardwareMap hardwareMap;

    public boolean llValid = true;
    public boolean hasTag = true;

    private static Vision instance;

    public Vision(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);

        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void loop() {
        if (limelight == null) {
            llValid = false;
            return;
        }

        llValid = true;
        hasTag = false;

        int goodTagId;

        if (Robot.alliance == Alliance.BLUE) {
            goodTagId = 20;
        } else if (Robot.alliance == Alliance.RED) {
            goodTagId = 24;
        } else {
            goodTagId = -1;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null) return;

        for (LLResultTypes.FiducialResult fidResult : result.getFiducialResults()) {


            if (fidResult.getFiducialId() == goodTagId) {
                goodTag = fidResult;
                hasTag = true;
            }
        }
    }

    public Optional<Double> getTx() {
        if (goodTag == null) return Optional.empty();

        return Optional.of(goodTag.getTargetXDegrees());
    }

    public Optional<Double> getTy() {
        if (goodTag == null) return Optional.empty();

        return Optional.of(goodTag.getTargetYDegrees());
    }

    public Optional<Double> getTa() {
        if (goodTag == null) return Optional.empty();

        return Optional.of(goodTag.getTargetArea());
    }

    public Optional<Double> getDistance() {
        if (getTy().isEmpty()) return Optional.empty();

        double ty = getTy().get();

        return Optional.of(1.47 + -.107 * ty + 7.98e-3 * Math.pow(ty, 2) + -3.05e-4 * Math.pow(ty, 3));
    }

    //NOTE: These do not work but there's something here
//    public Optional<Double> getHorizontalAngle() {
//        if (result == null || goodTag == null) return Optional.empty();
//
//        Pose3D tagPose = goodTag.getTargetPoseRobotSpace();
//
//        double x = tagPose.getPosition().x;
//        double y = tagPose.getPosition().y;
//
//        double horizontalAngleDegrees = Math.toDegrees(Math.atan2(y, x));
//
//        return Optional.of(horizontalAngleDegrees);
//    }
//
//    public Optional<Double> getVerticalAngle() {
//        if (result == null || goodTag == null) return Optional.empty();
//
//        Pose3D tagPose = goodTag.getTargetPoseCameraSpace();
//
//        double x = tagPose.getPosition().x;
//        double y = tagPose.getPosition().y;
//        double z = tagPose.getPosition().z;
//
//        double dist = Math.sqrt(x*x + y*y);
//
//        double verticalAngleRadians = Math.toDegrees(Math.atan2(z, dist));
//
//        return Optional.of(verticalAngleRadians);
//    }


    public static Vision getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new Vision(hardwareMap);
        }
        return instance;
    }

    public static Vision getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Vision not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }



}
