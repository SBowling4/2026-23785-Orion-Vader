package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;

@TeleOp(name = "TurretTest")
public class TurretTest extends OpMode {
    TurretSubsystem turretSubsystem;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, telemetry);

        turretSubsystem.init();
    }

    @Override
    public void loop() {
        telemetry.addData("Turret Position", turretSubsystem.getPosition());
        telemetry.addData("Turret Raw", turretSubsystem.getRawPosition());

        telemetry.update();
    }
}
