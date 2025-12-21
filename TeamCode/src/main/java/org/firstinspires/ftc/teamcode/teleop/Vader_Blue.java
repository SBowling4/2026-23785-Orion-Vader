package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Hood.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Vader_Blue", group = "Orion")
public class Vader_Blue extends OpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
//    HoodSubsystem hoodSubsystem;
//    Vision vision;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
//    TurretSubsystem turretSubsystem;



    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
//        hoodSubsystem = HoodSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
//        vision = Vision.getInstance(hardwareMap, telemetry);
//        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, telemetry);


//        vision.init();
        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
//        hoodSubsystem.init();
        feederSubsystem.init();
//        turretSubsystem.init();

        Robot.sendHardwareMap(hardwareMap);
    }

    @Override
    public void start() {
//        driveSubsystem.start();
//        vision.start();
    }

    @Override
    public void loop() {
        driveSubsystem.loop();
        intakeSubsystem.loop();
//        hoodSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
//        vision.loop();




        telemetry.update();
    }

}