package org.firstinspires.ftc.lib.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(38.3)
            .forwardZeroPowerAcceleration(-50.51033318645804)
            .lateralZeroPowerAcceleration(-71.35929610657753)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1, 0, .005, .04))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,.003,.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.07,0,.001,.6,0.01))
            .centripetalScaling(0.00019);

    public static PathConstraints pathConstraints = new PathConstraints(
            .99,
            100,
            .8,
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
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(57.86075592258057)
            .yVelocity(43.226006885091984);



    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants() //TODO: Something is wrong here and I will find it
            .forwardTicksToInches(5.405625457e-4)
            .strafeTicksToInches(5.5530193e-4)
            .turnTicksToInches(5.438790633e-4)
            .leftPodY(6.140628)
            .rightPodY(-6.140628)
            .strafePodX(-4.305)
            .leftEncoder_HardwareMapName(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_NAME)
            .rightEncoder_HardwareMapName(DriveConstants.RIGHT_BACK_MOTOR_NAME)
            .strafeEncoder_HardwareMapName(DriveConstants.RIGHT_FRONT_MOTOR_NAME)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();

    }


}