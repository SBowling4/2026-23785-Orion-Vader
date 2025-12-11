package org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

public class TurretSubsystem {
    private CRServoImplEx turretServo;
    private PIDController pidController;

    public double turretPower = 0.0;

    private Vision vision;

    private final HardwareMap hardwareMap;

    private static TurretSubsystem instance;


    private TurretSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void init() {
        turretServo = hardwareMap.get(CRServoImplEx.class, TurretConstants.TURRET_SERVO_NAME);

        pidController = new PIDController(
                TurretConstants.kP,
                TurretConstants.kI,
                TurretConstants.kD
        );

        vision = Vision.getInstance();
    }

    /**
     * Main loop: keep turret pointed at AprilTag.
     * Uses vision offset as an error signal (goal = 0°).
     */
    public void loop() {
        if (vision.getTx().isEmpty()) return;

        double tx = vision.getTx().get();

        double pidOut = pidController.calculate(tx, 0);

        turretPower = pidOut;

        turretServo.setPower(pidOut);
    }

    /**
     * Stop turret movement (hold current angle).
     */
    public void stop() {
        turretPower = 0;
        turretServo.setPower(0.0);
    }

    public static TurretSubsystem getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new TurretSubsystem(hardwareMap);
        }
        return instance;
    }

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("TurretSubsystem not initialized. Call getInstance(hardwareMap) first.");
        }
        return instance;
    }

}
