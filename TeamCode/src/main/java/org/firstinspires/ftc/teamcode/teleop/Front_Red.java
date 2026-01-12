package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;

@TeleOp(name = "Front_Red", group = "Orion")
public class Front_Red extends OpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    HoodSubsystem hoodSubsystem;
    Vision vision;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    TurretSubsystem turretSubsystem;



    @Override
    public void init() {
        Robot.alliance = Alliance.RED;
        Robot.sendHardwareMap(hardwareMap);
        Robot.lastPose = Robot.redFrontStart;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1);


        vision.init();
        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
        hoodSubsystem.init();
        feederSubsystem.init();
        turretSubsystem.init();
    }

    @Override
    public void start() {
        driveSubsystem.start();
        vision.start();
    }

    @Override
    public void loop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
        hoodSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
        vision.loop();
        turretSubsystem.loop();

        telemetry.addLine("Robot");
        telemetry.addData("Voltage", Robot.getRobotVoltage());

        driveSubsystem.setTelemetry(telemetry);
        intakeSubsystem.setTelemetry(telemetry);
        hoodSubsystem.setTelemetry(telemetry);
        flywheelSubsystem.setTelemetry(telemetry);
        feederSubsystem.setTelemetry(telemetry);
        vision.setTelemetry(telemetry);
        turretSubsystem.setTelemetry(telemetry);

        telemetry.update();
    }

    @Override
    public void stop() {
        Robot.lastPose = driveSubsystem.getFollowerPose();
        Robot.lastHood = hoodSubsystem.getPosition();
        Robot.lastTurret = turretSubsystem.getPosition();
    }
}