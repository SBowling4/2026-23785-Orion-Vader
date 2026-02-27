package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;

@Autonomous(name = "Front_Offline_Red")
public class Front_Offline extends BaseAuto {

    public enum PathState {
        DRIVE_OFFLINE,
        END
    }

    PathChain driveOffline;

    PathState pathState;

    @Override
    protected void buildPaths() {
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FRONT_START,
                DefinedPose.FRONT_ONLY_OFFLINE,
                getAlliance()
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(DefinedPose.FRONT_START.pose.mirror());
    }

    @Override
    protected void initializeFirstState() {
        pathState = PathState.DRIVE_OFFLINE;
    }

    @Override
    protected void updateStateMachine() {
        switch (pathState) {
            case DRIVE_OFFLINE:
                follower.followPath(driveOffline);

                pathState = PathState.END;
                resetPathTimer();
                break;
            case END:
                if (!follower.isBusy()) {
                    stopAllSubsystems();
                }
                break;
        }
    }


    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

}
