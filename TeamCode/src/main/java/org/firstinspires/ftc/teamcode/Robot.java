package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelConstants;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Configurable
public class Robot {
    public static Alliance alliance = Alliance.UNKNOWN;
    public static boolean tuningMode = false;

    public static ShooterStates shooterState = ShooterStates.MID;

    private static HardwareMap hardwareMap;


    public enum ShooterStates {
        FAR(HoodConstants.FAR_ANGLE, FlywheelConstants.FAR_VELOCITY),
        MID(HoodConstants.MID_ANGLE, FlywheelConstants.MID_VELOCITY),
        CLOSE(HoodConstants.CLOSE_ANGLE, FlywheelConstants.CLOSE_VELOCITY);
        public final double angle;
        public final double velocity;

        private ShooterStates(double angle, double velocity) {
            this.angle = angle;
            this.velocity = velocity;
        }
    }

    public static void advanceShooterState() {
        switch (shooterState) {
            case FAR:
                break;
            case MID:
                shooterState = ShooterStates.FAR;
                break;
            case CLOSE:
                shooterState = ShooterStates.MID;
                break;
        }
    }

    public static void reverseShooterState() {
        switch (shooterState) {
            case FAR:
                shooterState = ShooterStates.MID;
                break;
            case MID:
                shooterState = ShooterStates.CLOSE;
                break;
            case CLOSE:
                break;
        }
    }

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
