package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;

@Autonomous(name = "Huskie_Red", group = "Red")
public class Huskie extends BaseAuto {

    enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_READY_SECOND_PICKUP,
        DRIVE_SECOND_PICKUP,
        DRIVE_SECOND_GATE,
        DRIVE_SECOND_GATE_TO_SHOOT,
        SHOOT_SECOND_PICKUP,
        DRIVE_FIRST_PICKUP,
        DRIVE_FIRST_GATE,
        DRIVE_FIRST_GATE_TO_SHOOT,
        SHOOT_FIRST_PICKUP,
        END
    }

    private PathState pathState;
    private PathChain driveToShootPreload, driveSecondPickup, driveSecondPickupToGate, driveSecondGateToShoot, driveFirstPickup, driveFirstPickupToGate, driveFirstGateToShootOffline;

    // Flags to ensure we only call followPath once per state
    private boolean hasStartedDriveToShootPreload = false;

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
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

        driveSecondPickup = PathBuilder.buildPath(
                follower,
                .4,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.SECOND_PICKUP_CONTROL,
                DefinedPose.SECOND_PICKUP,
                getAlliance()
        );

        driveSecondPickupToGate = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.SECOND_PICKUP,
                DefinedPose.SECOND_PICKUP_GATE_CONTROL,
                DefinedPose.SECOND_GATE,
                getAlliance()
        );
        driveSecondGateToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.SECOND_GATE,
                DefinedPose.SECOND_GATE_TO_SHOOT_CONTROL,
                DefinedPose.FRONT_SHOOT,
                getAlliance()
        );
        driveFirstPickup = PathBuilder.buildPath(
                follower,
                .2,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.FIRST_PICKUP,
                getAlliance()
        );
        driveFirstPickupToGate = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FIRST_PICKUP,
                DefinedPose.FIRST_PICKUP_GATE_CONTROL,
                DefinedPose.FIRST_GATE,
                getAlliance()
        );
        driveFirstGateToShootOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FIRST_GATE,
                DefinedPose.OFFLINE_SHOOT,
                getAlliance()
        );
    }

    @Override
    protected void setStartingPose() {
        follower.setStartingPose(DefinedPose.FRONT_START.pose.mirror());
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
                if (!hasStartedDriveToShootPreload) {
                    follower.followPath(driveToShootPreload, .5 ,false);
                    hasStartedDriveToShootPreload = true;
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
                    pathState = PathState.DRIVE_SECOND_PICKUP;
                    resetPathTimer();
                    follower.followPath(driveSecondPickup, .8, false);
                }
                break;
            case DRIVE_SECOND_PICKUP:
                setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);

                if (!follower.isBusy()) {
                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                    pathState = PathState.DRIVE_SECOND_GATE;
                    resetPathTimer();
                    follower.followPath(driveSecondPickupToGate);
                }

                break;
            case DRIVE_SECOND_GATE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    pathState = PathState.DRIVE_SECOND_GATE_TO_SHOOT;
                    resetPathTimer();

                    follower.followPath(driveSecondGateToShoot);
                }

                break;
            case DRIVE_SECOND_GATE_TO_SHOOT:
                revFlywheel();
                stopIntake();

                if (!follower.isBusy()){
                    pathState = PathState.SHOOT_SECOND_PICKUP;
                    resetPathTimer();
                }

                break;
            case SHOOT_SECOND_PICKUP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_FIRST_PICKUP;
                    resetPathTimer();
                    follower.followPath(driveFirstPickup, .8, false);
                }
                break;
            case DRIVE_FIRST_PICKUP:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_FIRST_GATE;
                    resetPathTimer();
                    follower.followPath(driveFirstPickupToGate);
                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                }
                break;
            case DRIVE_FIRST_GATE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    revFlywheel();

                    pathState = PathState.DRIVE_FIRST_GATE_TO_SHOOT;
                    resetPathTimer();

                    follower.followPath(driveFirstGateToShootOffline);
                }
                break;
            case DRIVE_FIRST_GATE_TO_SHOOT:
                stopIntake();
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_FIRST_PICKUP;
                    resetPathTimer();
                }
                break;
            case SHOOT_FIRST_PICKUP:
                if (shootSequence()) {
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