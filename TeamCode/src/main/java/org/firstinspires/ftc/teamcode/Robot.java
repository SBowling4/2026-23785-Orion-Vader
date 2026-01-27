package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.lib.orion.util.Alliance;

@Configurable
public final class Robot {

    public static Alliance alliance = Alliance.UNKNOWN;
    public static boolean tuningMode = false;

    public static Pose lastPose = new Pose(9, 9, 270);

    private static HardwareMap hardwareMap;

    private Robot() {}

    public static void sendHardwareMap(HardwareMap hm) {
        hardwareMap = hm;
    }

    public static HardwareMap getHardwareMap() {
        if (hardwareMap == null) {
            throw new IllegalStateException("HardwareMap not sent to Robot");
        }
        return hardwareMap;
    }

    public static double getRobotVoltage() {
        return getHardwareMap()
                .voltageSensor
                .iterator()
                .next()
                .getVoltage();
    }
}

