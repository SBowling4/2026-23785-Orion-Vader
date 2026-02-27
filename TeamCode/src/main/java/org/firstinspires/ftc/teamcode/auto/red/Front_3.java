package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;

@Autonomous(name = "Front_3_Red", group = "Blue")
public class Front_3 extends BaseAuto {

    enum PathState {
        DRIVE_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;
    private PathChain driveToShoot, driveOffline;

    // Flags to ensure we only call followPath once per state
    private boolean hasStartedDriveToShoot = false;
    private boolean hasStartedDriveOffline = false;

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void buildPaths() {
        driveToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FRONT_START,
                DefinedPose.FRONT_SHOOT,
                getAlliance()
        );
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.CONSTANT,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.GATE_OFFLINE,
                getAlliance()
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(DefinedPose.FRONT_START.pose.mirror());
    }

    @Override
    protected void initializeFirstState() {
        pathState = PathState.DRIVE_TO_SHOOT;
        resetPathTimer();
    }

    @Override
    protected void updateStateMachine() {
        switch (pathState) {
            case DRIVE_TO_SHOOT:
                flywheelSubsystem.setPower(0.35);

                // Start path only once
                if (!hasStartedDriveToShoot) {
                    follower.followPath(driveToShoot, .5, false);
                    hasStartedDriveToShoot = true;
                }

                // Wait for path to complete before shooting
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_PRELOAD;
                    resetPathTimer();
                }
                break;

            case SHOOT_PRELOAD:
                // shootSequence() handles everything and returns true when done
                if (shootSequence()) {
                    pathState = PathState.DRIVE_OFFLINE;
                    resetPathTimer();
                }
                break;

            case DRIVE_OFFLINE:
                // Start path only once
                if (!hasStartedDriveOffline) {
                    follower.followPath(driveOffline);
                    hasStartedDriveOffline = true;
                }

                // Wait for path to complete
                if (!follower.isBusy()) {
                    pathState = PathState.END;
                    resetPathTimer();
                }
                break;

            case END:
                stopAllSubsystems();
                break;
        }
    }
}