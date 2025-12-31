package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.lib.orion.util.Alliance;

@Configurable
public class Robot {
    public static Alliance alliance = Alliance.UNKNOWN;

    public static boolean tuningMode = false;

    private static HardwareMap hardwareMap;


    public static void sendHardwareMap(HardwareMap hm) {
        hardwareMap = hm;
    }

    public static double getRobotVoltage() {
        try {
            for (VoltageSensor vs : hardwareMap.voltageSensor) {
                return vs.getVoltage();
            }
        } catch (NullPointerException npe) {
            throw new NullPointerException("Hardware map not sent to Robot");
        }

        return -1;
    }



}
