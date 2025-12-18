package org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeConstants {
    public static final String INTAKE_MOTOR_NAME = "intake";

    public enum INTAKE_STATE {
        INTAKE(1),
        OUT(-1),
        STOP(0);

        private double power;

        private INTAKE_STATE(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }
}
