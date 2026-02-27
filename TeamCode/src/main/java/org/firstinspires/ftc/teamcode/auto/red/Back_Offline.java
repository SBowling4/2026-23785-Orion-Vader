package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;

@Autonomous(name = "Back_Offline_Red")
public class Back_Offline extends BaseAuto {
    enum PathState {
        DRIVE_OFFLINE,
        END
    }

    PathState pathState;

    PathChain driveOffline;

    @Override
    protected void buildPaths() {
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.CONSTANT,
                DefinedPose.BACK_START,
                DefinedPose.BACK_OFFLINE,
                getAlliance()
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(DefinedPose.BACK_START.pose.mirror());
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
        }
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }


}
