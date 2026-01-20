package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.pedroPathing.Constants;

@Autonomous(name = "Front_Offline_Red")
public class Offline extends OpMode {
    private Follower follower;

    Pose startPose = new Pose(126.00722021660651, 121.15523465703973, Math.toRadians(36));
    Pose endPose = new Pose(94.64259927797835, 120.90974729241873, Math.toRadians(0));

    PathChain path;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();

    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();
    }
}
