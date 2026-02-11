package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.Poses;

@Autonomous(name = "FrontOffline_Blue")
public class FrontOffline extends BaseAuto {

    public enum PathState {
        DRIVE_OFFLINE,
        END
    }

    PathChain driveOffline;

    PathState pathState;

    @Override
    protected void buildPaths() {
        driveOffline = PathBuilder.buildPath(follower, PathBuilder.Heading.LINEAR, Poses.FRONT_START, Poses.FRONT_ONLY_OFFLINE);
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(Poses.FRONT_START);
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
        return Alliance.BLUE;
    }

    @Override
    protected void updateTelemetry() {
        super.updateTelemetry();
        addStateTelemetry(pathState);
    }
}
