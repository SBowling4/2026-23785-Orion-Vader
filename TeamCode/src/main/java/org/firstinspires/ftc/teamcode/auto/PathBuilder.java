package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.lib.orion.util.Alliance;

public class PathBuilder {
    public static PathChain buildPath(Follower follower, Heading heading, DefinedPose startPose, DefinedPose endPose, Alliance alliance) {
        if (heading == Heading.CONSTANT) {
            return follower.pathBuilder()
                    .addPath(alliance == Alliance.BLUE ? new BezierLine(startPose.pose, endPose.pose) : new BezierLine(startPose.pose.mirror(), endPose.pose.mirror()))
                    .setConstantHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading())
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(alliance == Alliance.BLUE ? new BezierLine(startPose.pose, endPose.pose) : new BezierLine(startPose.pose.mirror(), endPose.pose.mirror()))
                    .setLinearHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading(), alliance == Alliance.BLUE ? endPose.pose.getHeading() : endPose.pose.mirror().getHeading())
                    .build();

        }
    }

    public static PathChain buildPath(Follower follower, Heading heading, DefinedPose startPose, DefinedPose controlPoint, DefinedPose endPose, Alliance alliance) {
        if (heading == Heading.CONSTANT) {
            return follower.pathBuilder()
                    .addPath(alliance == Alliance.BLUE ? new BezierCurve(startPose.pose, controlPoint.pose, endPose.pose) : new BezierCurve(startPose.pose.mirror(), controlPoint.pose.mirror(), endPose.pose.mirror()))
                    .setConstantHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading())
                    .build();
        } else {
            return follower.pathBuilder()
                    .addPath(alliance == Alliance.BLUE ? new BezierCurve(startPose.pose, controlPoint.pose, endPose.pose) : new BezierCurve(startPose.pose.mirror(), controlPoint.pose.mirror(), endPose.pose.mirror()))
                    .setLinearHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading(), alliance == Alliance.BLUE ? endPose.pose.getHeading() : endPose.pose.mirror().getHeading())
                    .build();

        }
    }

    public static PathChain buildPath(Follower follower, double headingEnd, DefinedPose startPose, DefinedPose endPose, Alliance alliance) {
        return follower.pathBuilder()
                .addPath(alliance == Alliance.BLUE ? new BezierLine(startPose.pose, endPose.pose) : new BezierLine(startPose.pose.mirror(), endPose.pose.mirror()))
                .setLinearHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading(), alliance == Alliance.BLUE ? endPose.pose.getHeading() : endPose.pose.mirror().getHeading(), headingEnd)
                .build();
    }

    public static PathChain buildPath(Follower follower, double headingEnd, DefinedPose startPose, DefinedPose controlPoint, DefinedPose endPose, Alliance alliance) {
        return follower.pathBuilder()
                .addPath(alliance == Alliance.BLUE ? new BezierCurve(startPose.pose, controlPoint.pose, endPose.pose) : new BezierCurve(startPose.pose.mirror(), controlPoint.pose.mirror(), endPose.pose.mirror()))
                .setLinearHeadingInterpolation(alliance == Alliance.BLUE ? startPose.pose.getHeading() : startPose.pose.mirror().getHeading(), alliance == Alliance.BLUE ? endPose.pose.getHeading() : endPose.pose.mirror().getHeading(), headingEnd)
                .build();
    }





    public enum Heading {
        CONSTANT,
        LINEAR
    }
}
