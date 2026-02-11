package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.orion.BaseOpMode;
import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.VisionSubsystem;

@TeleOp(name = "Vader_Red", group = "Orion")
public class Vader_Red extends BaseOpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    //    HoodSubsystem hoodSubsystem;
    VisionSubsystem visionSubsystem;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    TurretSubsystem turretSubsystem;

    public Vader_Red() {
        super(Alliance.RED);
    }

    @Override
    public void onInit() {
        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
//        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        visionSubsystem = VisionSubsystem.getInstance(hardwareMap, gamepad1);
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);


        visionSubsystem.init();
        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
//        hoodSubsystem.init();
        feederSubsystem.init();
        turretSubsystem.init();
    }

    @Override
    public void onStart() {
        driveSubsystem.start();
        visionSubsystem.start();
    }

    @Override
    public void onLoop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
//        hoodSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
        visionSubsystem.loop();
        turretSubsystem.loop();



        TelemetryPacket packet = new TelemetryPacket();

        driveSubsystem.setTelemetry(packet);
        intakeSubsystem.setTelemetry(packet);
//        hoodSubsystem.setTelemetry(packet);
        flywheelSubsystem.setTelemetry(packet);
        feederSubsystem.setTelemetry(packet);
        visionSubsystem.setTelemetry(packet);
        turretSubsystem.setTelemetry(packet);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void onStop() {
        Robot.lastPose = driveSubsystem.getFollowerPose();
    }
}