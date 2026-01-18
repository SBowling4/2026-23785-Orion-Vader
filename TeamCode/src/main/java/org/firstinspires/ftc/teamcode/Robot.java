package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

@Configurable
public class Robot {
    public static Alliance alliance = Alliance.UNKNOWN;

    public static boolean tuningMode = false;

    public static Pose lastPose = new Pose(72,72,0);

    public static HardwareMap hardwareMap;

    public static void sendHardwareMap(HardwareMap hm) {
        Robot.hardwareMap = hm;
    }

    public static double getRobotVoltage() {
        try {
            Robot.hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (NullPointerException npe) {
            throw new NullPointerException("Hardware map not sent to Robot");
        }

        throw new IllegalStateException("");
    }



}
