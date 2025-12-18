package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drive.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Feeder.FeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Vader_Blue", group = "Orion")
public class Vader_Blue extends OpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
//    ShooterSubsystem shooterSubsystem;
//    Vision vision;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;
    TurretSubsystem turretSubsystem;

    private boolean lastUpState = false;
    private boolean lastDownState = false;


    @Override
    public void init() {
        Robot.alliance = Alliance.BLUE;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
//        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1, telemetry);
//        vision = Vision.getInstance(hardwareMap, telemetry);
        turretSubsystem = TurretSubsystem.getInstance(hardwareMap, gamepad1, telemetry);


//        vision.init();
        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
//        shooterSubsystem.init();
        feederSubsystem.init();
        turretSubsystem.init();

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
//        shooterSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
//        vision.loop();
//        turretSubsystem.loop();

//        turretSubsystem.setTurretPower(0);

//        boolean currentUpState = gamepad1.dpad_up;
//        boolean currentDownState = gamepad1.dpad_down;
//
//        if (currentUpState && !lastUpState) {
//            Robot.advanceShooterState();
//        }
//
//        if (currentDownState && !lastDownState) {
//            Robot.reverseShooterState();
//        }
//
//        lastUpState = currentUpState;
//        lastDownState = currentDownState;
//
//        if (!Robot.tuningMode && !gamepad1.right_bumper) {
//            if (gamepad1.left_bumper) {
//                flywheelSubsystem.setVelocity(Robot.shooterState.velocity);
//                shooterSubsystem.setAngle(Robot.shooterState.angle);
//            } else {
//                flywheelSubsystem.stop();
//                shooterSubsystem.setAngle(0);
//            }
//        }



        telemetry.update();
    }

}