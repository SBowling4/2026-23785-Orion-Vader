package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathBuilder {
    public static PathChain buildPath(Follower follower, Heading heading, Pose startPose, Pose endPose) {
        if (heading == Heading.CONSTANT) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setConstantHeadingInterpolation(startPose.getHeading())
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                    .build();
        }
    }

    public static PathChain buildPath(Follower follower, Heading heading, Pose startPose, Pose controlPoint, Pose endPose) {
        if (heading == Heading.CONSTANT) {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, controlPoint, endPose))
                    .setConstantHeadingInterpolation(startPose.getHeading())
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(startPose, controlPoint, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                    .build();
        }
    }



    public enum Heading {
        CONSTANT,
        LINEAR
    }
}
