package org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeConstants {
    public static final String INTAKE_MOTOR_NAME = "intake";

    public enum intakeState {
        INTAKE(-1),
        OUT(1),
        STOP(0);

        private double power;

        private intakeState(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }
    }
}
