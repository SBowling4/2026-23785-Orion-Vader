package org.firstinspires.ftc.lib.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(30)
            .forwardZeroPowerAcceleration(-72.78069448430112)
            .lateralZeroPowerAcceleration(-64.8184983519383);

    public static PathConstraints pathConstraints = new PathConstraints(
            .99,
            100,
            1,
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
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(68.98701234066769)
            .yVelocity(43.998690576845966);



    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants() //TODO: Something is wrong here and I will find it
            .forwardTicksToInches(5.2843826E-4)
            .strafeTicksToInches(5.2843826E-4)
            .turnTicksToInches(5.2843826E-4)
            .leftPodY(-6.140628)
            .rightPodY(6.140628)
            .strafePodX(-4.195)
            .leftEncoder_HardwareMapName(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME)
            .rightEncoder_HardwareMapName(DriveConstants.RIGHT_BACK_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();

    }


}