package org.firstinspires.ftc.teamcode.subsystems.Feeder;

public class FeederConstants {
    public static final String FEEDER_MOTOR_NAME = "feeder";
    public static final String KICKER_SERVO_NAME = "kicker";
    public static final String STOPPER_SERVO_NAME = "stopper";

    public enum feederState {
        IN(1),
        OUT(-1),
        STOP(0);

        private final double power;

        private feederState(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }

    }

    public enum STOPPER_STATE {
        OPEN(.76),
        CLOSED(1);

        private final double position;

        private STOPPER_STATE(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

    }

    public enum KICKER_STATE {
        IN(.1),
        OUT(.327);

        private final double position;

        private KICKER_STATE(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }

    }

}
