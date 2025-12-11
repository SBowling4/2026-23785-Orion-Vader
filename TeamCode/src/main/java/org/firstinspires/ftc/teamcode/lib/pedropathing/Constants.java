package org.firstinspires.ftc.teamcode.lib.pedropathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(30.6)
            .forwardZeroPowerAcceleration(-42.233538015420116)
            .lateralZeroPowerAcceleration(-114.49997897772923)
            .translationalPIDFCoefficients(new PIDFCoefficients(.15, 0, .002, .004))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, .02, .03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.02, 0, .00001, .6, .03))
            .centripetalScaling(0.0003);

    public static PathConstraints pathConstraints = new PathConstraints(
            .99,
            100,
            1.5,
            1
    );

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .rightRearMotorName(DriveConstants.RIGHT_BACK_MOTOR_NAME)
            .leftFrontMotorName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .leftRearMotorName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.88012216535767)
            .yVelocity(56.218767813481534);


    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(-5.562e-4)
            .strafeTicksToInches(5.597e-4)
            .turnTicksToInches(5.376443633716166E-4)
            .leftPodY(-6.140628)
            .rightPodY(6.140628)
            .strafePodX(-2.139)
            .leftEncoder_HardwareMapName(DriveConstants.LEFT_BACK_MOTOR_NAME)
            .rightEncoder_HardwareMapName(IntakeConstants.INTAKE_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(DriveConstants.LEFT_FRONT_MOTOR_NAME)
            .rightEncoderDirection(Encoder.FORWARD)
            .leftEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();

    }


}