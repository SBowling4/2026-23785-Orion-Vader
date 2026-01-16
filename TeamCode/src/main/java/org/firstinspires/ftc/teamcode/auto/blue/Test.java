package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.pedroPathing.Constants;

@Autonomous(name = "Test")
public class Test extends OpMode {
    private Follower follower;
    private Pose startPose = new Pose(72.25992779783394, 39.33574007220217, Math.toRadians(90));
    private Pose endPose = new Pose(71.91335740072202, 78.84476534296027, Math.toRadians(90));

    private PathChain path;

    private enum PathStates {
        START,
        END
    }

    private void buildPaths() {
        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
    }


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getHeading());
    }
}
