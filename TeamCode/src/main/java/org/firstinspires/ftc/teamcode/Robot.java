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

    public static Pose blueBackStart = new Pose(144 - 9.010830324909742, 57.1841155234657, Units.degreesToRadians(90 + 90));
    public static Pose blueFrontStart = new Pose(144 - 124.4187725631769, 144 - 120.60649819494586, Units.degreesToRadians(144 + 90));

    public static Pose redBackStart = new Pose(144 - 8.664259927797834,    86.98916967509025, Units.degreesToRadians(90 + 90));
    public static Pose redFrontStart = new Pose(144 - 124.4187725631769, 121.9927797833935, Units.degreesToRadians(121));

    public static double lastHood = 0;
    public static double lastTurret = 0;

    public static HardwareMap hardwareMap;

    public static RobotMode mode = RobotMode.VADER;

    public enum RobotMode {
        KAOS,
        VADER
    }

    public static void sendHardwareMap(HardwareMap hm) {
        Robot.hardwareMap = hm;
    }

    public static double getRobotVoltage() {
        try {
            for (VoltageSensor vs : Robot.hardwareMap.voltageSensor) {
                return vs.getVoltage();
            }
        } catch (NullPointerException npe) {
            throw new NullPointerException("Hardware map not sent to Robot");
        }

        throw new IllegalStateException("");
    }



}
