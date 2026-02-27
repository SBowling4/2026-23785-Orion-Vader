package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;

@Autonomous(name = "Front_6_Gate_Blue", group = "Blue")
public class Front_6_Gate extends BaseAuto {

    enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_PICKUP,
        GATE,
        DRIVE_GATE_TO_SHOOT,
        SHOOT_PICKUP,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;
    private PathChain driveToShootPreload, drivePickup, driveGateToShoot, driveOffline, driveGate;

    // Flags to ensure we only call followPath once per state
    private boolean hasStartedDriveToShoot = false;

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected void buildPaths() {
        driveToShootPreload = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FRONT_START,
                DefinedPose.FRONT_SHOOT,
                getAlliance()
        );
        drivePickup = PathBuilder.buildPath(
                follower,
                .2,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.FIRST_PICKUP,
                getAlliance()
        );
        driveGate = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.CONSTANT,
                DefinedPose.FIRST_PICKUP,
                DefinedPose.FIRST_PICKUP_GATE_CONTROL,
                DefinedPose.FIRST_GATE,
                getAlliance()
        );
        driveGateToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FIRST_GATE,
                DefinedPose.FRONT_SHOOT,
                getAlliance()
        );
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.GATE_OFFLINE,
                getAlliance()
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(DefinedPose.FRONT_START.pose);
    }

    @Override
    protected void initializeFirstState() {
        pathState = PathState.DRIVE_START_TO_SHOOT;
        resetPathTimer();
    }

    @Override
    protected void updateStateMachine() {
        switch (pathState) {
            case DRIVE_START_TO_SHOOT:
                revFlywheel();

                // Start path only once
                if (!hasStartedDriveToShoot) {
                    follower.followPath(driveToShootPreload, .5, false);
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
                    pathState = PathState.DRIVE_PICKUP;
                    resetPathTimer();
                    follower.followPath(drivePickup, .8, false);
                }
                break;
            case DRIVE_PICKUP:
                setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);

                if (!follower.isBusy()) {
                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                    pathState = PathState.GATE;
                    resetPathTimer();
                    follower.followPath(driveGate);
                }

                break;
            case GATE:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_GATE_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(driveGateToShoot);
                }

            case DRIVE_GATE_TO_SHOOT:
                revFlywheel();

                if (!follower.isBusy()){
                    pathState = PathState.SHOOT_PICKUP;
                    resetPathTimer();
                }

                break;
            case SHOOT_PICKUP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_OFFLINE;
                    resetPathTimer();
                    follower.followPath(driveOffline);
                }
                break;
            case DRIVE_OFFLINE:
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