package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;

public class TurretTest extends OpMode {
    TurretSubsystem turretSubsystem;

    @Override
    public void init() {
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1);

        turretSubsystem.init();
    }

    @Override
    public void loop() {
        telemetry.addData("Turret Position", turretSubsystem.getPosition());
        telemetry.addData("Turret Raw", turretSubsystem.getRawPosition());
    }
}
