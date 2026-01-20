package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.pedroPathing.Constants;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

@Autonomous(name = "Front_Offline_Blue")
public class Front_Offline extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(17.769911504424773, 121.41592920353982, Units.degreesToRadians(144));
    Pose endPose = new Pose(49.06859205776173, 131.0324909747292, Math.toRadians(180));

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
