package org.firstinspires.ftc.lib.orion.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Custom IMU wrapper that allows software-defined yaw resetting
 * while remaining robust to IMU resets and jumps.
 */
public class OrionIMU {
    private final IMU internalIMU;

    /**
     * Software yaw offset, stored internally in RADIANS.
     */
    private double yawOffsetRad = 0.0;

    public OrionIMU(
            HardwareMap hardwareMap,
            RevHubOrientationOnRobot.LogoFacingDirection logo,
            RevHubOrientationOnRobot.UsbFacingDirection usb
    ) {
        internalIMU = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logo, usb)
        );

        internalIMU.initialize(parameters);
    }

    /**
     * Gets the current yaw in radians.
     */
    public double getYaw() {
        return yawOffsetRad + internalIMU
                .getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
    }

    /**
     * Gets the current yaw in the requested angle unit.
     */
    public double getYaw(AngleUnit unit) {
        double offset = (unit == AngleUnit.RADIANS)
                ? yawOffsetRad
                : Math.toDegrees(yawOffsetRad);

        return offset + internalIMU
                .getRobotYawPitchRollAngles()
                .getYaw(unit);
    }

    /**
     * Resets yaw so that the current heading becomes 0.
     */
    public void resetYaw() {
        yawOffsetRad = 0.0;
        internalIMU.resetYaw();
    }

    /**
     * Resets yaw so that the current heading becomes the specified angle.
     *
     * @param angleRadians desired yaw after reset, in RADIANS
     */
    public void resetYaw(double angleRadians) {
        yawOffsetRad = angleRadians;
        internalIMU.resetYaw();
    }

    /**
     * Gets yaw normalized to [0, 2π).
     */
    public double getAbsoluteYaw() {
        return normalize(getYaw(), AngleUnit.RADIANS);
    }

    /**
     * Gets yaw normalized to [0, 360) or [0, 2π).
     */
    public double getAbsoluteYaw(AngleUnit unit) {
        return normalize(getYaw(unit), unit);
    }

    /**
     * Normalizes an angle to [0, 360) or [0, 2π).
     */
    private double normalize(double angle, AngleUnit unit) {
        double modulus = (unit == AngleUnit.RADIANS)
                ? 2 * Math.PI
                : 360.0;

        angle %= modulus;
        if (angle < 0) angle += modulus;
        return angle;
    }
}
