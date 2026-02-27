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

@Autonomous(name = "Back_6_Red", group = "Blue")
public class Back_6 extends BaseAuto {

    enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_PICKUP,
        DRIVE_PICKUP_TO_SHOOT,
        SHOOT_PICKUP,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;
    private PathChain driveToShootPreload, drivePickup, drivePickupToShoot, driveOffline;

    // Flags to ensure we only call followPath once per state
    private boolean hasStartedDriveToShoot = false;

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void buildPaths() {
        driveToShootPreload = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.BACK_START,
                DefinedPose.BACK_SHOOT,
                getAlliance()
        );
        drivePickup = PathBuilder.buildPath(
                follower,
                .4,
                DefinedPose.BACK_SHOOT,
                DefinedPose.THIRD_PICKUP_BACK_CONTROL,
                DefinedPose.THIRD_PICKUP,
                getAlliance()
        );

        drivePickupToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.THIRD_PICKUP,
                DefinedPose.BACK_SHOOT,
                getAlliance()
        );
        driveOffline = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.BACK_SHOOT,
                DefinedPose.BACK_OFFLINE,
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
                    follower.followPath(drivePickup);
                }
                break;
            case DRIVE_PICKUP:
                setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);

                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_PICKUP_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(drivePickupToShoot);
                }

                break;

            case DRIVE_PICKUP_TO_SHOOT:
                revFlywheel();

                if (!follower.isBusy()){
                    pathState = PathState.SHOOT_PICKUP;
                    resetPathTimer();
                    stopIntake();
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