package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.Poses;

@Autonomous(name = "Front_3_Blue", group = "Blue")
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
        return Alliance.BLUE;
    }

    @Override
    protected void buildPaths() {
        driveToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                Poses.FRONT_START,
                Poses.SHOOT_POSE
        );
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.CONSTANT,
                Poses.SHOOT_POSE,
                Poses.GATE_OFFLINE
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(Poses.FRONT_START);
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
                    follower.followPath(driveToShoot);
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

    @Override
    protected void updateTelemetry() {
        super.updateTelemetry();
        addStateTelemetry(pathState);
    }
}