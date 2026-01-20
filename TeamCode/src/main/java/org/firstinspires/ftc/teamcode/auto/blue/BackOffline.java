package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.pedroPathing.Constants;

@Autonomous(name = "Back_Offline_Blue")
public class BackOffline extends OpMode {
    private Follower follower;

    Pose startPose = new Pose(44.56317689530686, 8.693140794223787, Math.toRadians(180));
    Pose endPose = new Pose(22.209386281588447, 8.447653429602884, Math.toRadians(180));

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
