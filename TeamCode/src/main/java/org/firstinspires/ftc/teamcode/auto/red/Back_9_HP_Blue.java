package org.firstinspires.ftc.teamcode.auto.red;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.auto.BaseAuto;
import org.firstinspires.ftc.teamcode.auto.DefinedPose;
import org.firstinspires.ftc.teamcode.auto.PathBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;

@Autonomous(name = "Back_9_HP_Red", group = "Blue")
public class Back_9_HP_Blue extends BaseAuto {

    enum PathState {
        DRIVE_START_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        DRIVE_THIRD_PICKUP,
        DRIVE_THIRD_PICKUP_TO_SHOOT,
        SHOOT_THIRD_PICKUP,
        DRIVE_HP_PICKUP,
        DRIVE_HP_TO_SHOOT,
        SHOOT_HP,
        DRIVE_OFFLINE,
        END
    }

    private PathState pathState;
    private PathChain driveToShootPreload, driveThirdPickup, driveThirdPickupToShoot, driveHPPickup, driveHPToShoot, driveOffline;

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
        driveThirdPickup = PathBuilder.buildPath(
                follower,
                .4,
                DefinedPose.BACK_SHOOT,
                DefinedPose.THIRD_PICKUP_BACK_CONTROL,
                DefinedPose.THIRD_PICKUP,
                getAlliance()
        );
        driveThirdPickupToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.THIRD_PICKUP,
                DefinedPose.BACK_SHOOT,
                getAlliance()
        );
        driveHPPickup = follower.pathBuilder()
                .addPath(new BezierLine(DefinedPose.BACK_SHOOT.pose, DefinedPose.HP_PICKUP.pose))
                .setLinearHeadingInterpolation(DefinedPose.BACK_SHOOT.pose.getHeading(), DefinedPose.HP_PICKUP.pose.getHeading(), .4)
                .build();
        driveHPToShoot = PathBuilder.buildPath(
                follower,
                PathBuilder.Heading.LINEAR,
                DefinedPose.HP_PICKUP,
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
        pathState = PathState.DRIVE_START_TO_SHOOT_PRELOAD;
        resetPathTimer();
    }

    @Override
    protected void updateStateMachine() {
        switch (pathState) {
            case DRIVE_START_TO_SHOOT_PRELOAD:
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
                    pathState = PathState.DRIVE_THIRD_PICKUP;
                    resetPathTimer();
                    follower.followPath(driveThirdPickup);
                }
                break;
            case DRIVE_THIRD_PICKUP:
                setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);

                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_THIRD_PICKUP_TO_SHOOT;
                    resetPathTimer();
                    follower.followPath(driveThirdPickupToShoot);
                }

                break;

            case DRIVE_THIRD_PICKUP_TO_SHOOT:
                revFlywheel();

                if (!follower.isBusy()){
                    pathState = PathState.SHOOT_THIRD_PICKUP;
                    resetPathTimer();
                }

                break;
            case SHOOT_THIRD_PICKUP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_HP_PICKUP;
                    resetPathTimer();
                    setFeederAndIntakeState(IntakeConstants.intakeState.INTAKE, FeederConstants.feederState.IN);
                    follower.followPath(driveHPPickup);
                }
                break;
            case DRIVE_HP_PICKUP:
                if (!follower.isBusy()) {
                    pathState = PathState.DRIVE_HP_TO_SHOOT;
                    resetPathTimer();

                    setFeederAndIntakeState(IntakeConstants.intakeState.STOP, FeederConstants.feederState.IN);
                    follower.followPath(driveHPToShoot);
                }
                break;
            case DRIVE_HP_TO_SHOOT:
                if (!follower.isBusy()) {
                    stopIntake();

                    pathState = PathState.SHOOT_HP;
                    resetPathTimer();
                }
                break;
            case SHOOT_HP:
                if (shootSequence()) {
                    pathState = PathState.DRIVE_OFFLINE;
                    resetPathTimer();

                    follower.followPath(driveOffline);
                }
                break;
            case DRIVE_OFFLINE:
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