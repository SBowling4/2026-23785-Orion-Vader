package org.firstinspires.ftc.teamcode.auto.blue;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;

@Autonomous(name = "Front_12_Blue", group = "Blue")
public class Front_12 extends BaseAuto {

    enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_FIRST_PICKUP,
        DRIVE_FIRST_PICKUP_TO_SHOOT,
        SHOOT_FIRST_PICKUP,
        DRIVE_SECOND_PICKUP,
        DRIVE_SECOND_PICKUP_TO_SHOOT,
        SHOOT_SECOND_PICKUP,
        DRIVE_THIRD_PICKUP,
        DRIVE_THIRD_PICKUP_TO_SHOOT,
        SHOOT_THIRD_PICKUP,
        END
    }

    private PathState pathState;
    private PathChain driveToShootPreload, driveFirstPickup, driveFirstPickupToShoot, driveSecondPickup, driveSecondPickupToShoot, driveThirdPickup, driveThirdPickupToShootOffline;

    // Flags to ensure we only call followPath once per state
    private boolean hasStartedDriveToShootPreload = false;

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
        driveFirstPickup = PathBuilder.buildPath(
                follower,
                .2,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.FIRST_PICKUP,
                getAlliance()
        );
        driveFirstPickupToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FIRST_PICKUP,
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
        driveSecondPickupToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.SECOND_PICKUP,
                DefinedPose.SECOND_PICKUP_CONTROL,
                DefinedPose.FRONT_SHOOT,
                getAlliance()
        );
        driveThirdPickup = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.FRONT_SHOOT,
                DefinedPose.THIRD_PICKUP_FRONT_CONTROL,
                DefinedPose.THIRD_PICKUP,
                getAlliance()
        );
        driveThirdPickupToShootOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.THIRD_PICKUP,
                DefinedPose.OFFLINE_SHOOT,
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
                if (!hasStartedDriveToShootPreload) {
                    follower.followPath(driveToShootPreload, .5, false);
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
                    pathState = PathState.DRIVE_FIRST_PICKUP;
                    resetPathTimer();
                    follower.followPath(driveFirstPickup, .8, false);
                }
                break;
            case DRIVE_FIRST_PICKUP:
                setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);

                if (!follower.isBusy()) {
                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                    pathState = PathState.DRIVE_FIRST_PICKUP_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(driveFirstPickupToShoot);
                }

                break;

            case DRIVE_FIRST_PICKUP_TO_SHOOT:
                revFlywheel();

                if (!follower.isBusy()){
                    pathState = PathState.SHOOT_FIRST_PICKUP;
                    resetPathTimer();
                }

                break;
            case SHOOT_FIRST_PICKUP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_SECOND_PICKUP;
                    resetPathTimer();
                    follower.followPath(driveSecondPickup, .8, false);
                    setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);
                }
                break;
            case DRIVE_SECOND_PICKUP:
                if (!follower.isBusy()) {
                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                    revFlywheel();

                    pathState = PathState.DRIVE_SECOND_PICKUP_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(driveSecondPickupToShoot);
                }
                break;
            case DRIVE_SECOND_PICKUP_TO_SHOOT:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_SECOND_PICKUP;
                    resetPathTimer();
                }
                break;
            case SHOOT_SECOND_PICKUP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_THIRD_PICKUP;
                    resetPathTimer();
                    setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);
                    follower.followPath(driveThirdPickup);
                }
                break;
            case DRIVE_THIRD_PICKUP:
                if (!follower.isBusy()) {
                    stopIntake();
                    revFlywheel();

                    pathState = PathState.DRIVE_THIRD_PICKUP_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(driveThirdPickupToShootOffline);
                }
                break;
            case DRIVE_THIRD_PICKUP_TO_SHOOT:
                if (!follower.isBusy()) {
                    pathState = PathState.SHOOT_THIRD_PICKUP;
                    resetPathTimer();
                }
                break;
            case SHOOT_THIRD_PICKUP:
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