package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.lib.orion.util.Alliance;

@TeleOp
public class VisionTest extends OpMode {
    Vision vision;
    DriveSubsystem driveSubsystem;
    TurretSubsystem turretSubsystem;

    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;

        vision = Vision.getInstance(hardwareMap);
        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1);

        driveSubsystem.init();
        vision.init();
        turretSubsystem.init();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        vision.loop();
        driveSubsystem.loop();

        turretSubsystem.setPosition(0);

        vision.setTelemetry(telemetry);
    }
}
