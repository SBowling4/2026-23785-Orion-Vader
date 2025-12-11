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
import org.firstinspires.ftc.teamcode.subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Kaos_Red", group = "Orion")
public class Kaos_Red extends OpMode {
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    Vision vision;
    FlywheelSubsystem flywheelSubsystem;
    FeederSubsystem feederSubsystem;

    private boolean lastUpState = false;
    private boolean lastDownState = false;


    @Override
    public void init() {
        Robot.alliance = Alliance.RED;

        telemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        driveSubsystem = DriveSubsystem.getInstance(hardwareMap, gamepad1);
        intakeSubsystem = IntakeSubsystem.getInstance(hardwareMap, gamepad1);
        flywheelSubsystem = FlywheelSubsystem.getInstance(hardwareMap, gamepad1);
        shooterSubsystem = ShooterSubsystem.getInstance(hardwareMap, gamepad1, gamepad2);
        feederSubsystem = FeederSubsystem.getInstance(hardwareMap, gamepad1);
        vision = Vision.getInstance(hardwareMap);



        driveSubsystem.init();
        intakeSubsystem.init();
        flywheelSubsystem.init();
        shooterSubsystem.init();
        feederSubsystem.init();
        vision.init();

        Robot.sendHardwareMap(hardwareMap);
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
        shooterSubsystem.loop();
        flywheelSubsystem.loop();
        feederSubsystem.loop();
        vision.loop();

        boolean currentUpState = gamepad1.dpad_up;
        boolean currentDownState = gamepad1.dpad_down;

        if (currentUpState && !lastUpState) {
            Robot.advanceShooterState();
        }

        if (currentDownState && !lastDownState) {
            Robot.reverseShooterState();
        }

        lastUpState = currentUpState;
        lastDownState = currentDownState;

        if (!Robot.tuningMode && !gamepad1.right_bumper) {
            if (gamepad1.left_bumper) {
                flywheelSubsystem.setVelocity(Robot.shooterState.velocity);
                shooterSubsystem.setAngle(Robot.shooterState.angle);
            } else {
                flywheelSubsystem.stop();
                shooterSubsystem.setAngle(0);
            }
        }



        telemetry.addLine("//Odometry//");
        telemetry.addData("X", driveSubsystem.getPose().getX());
        telemetry.addData("Y", driveSubsystem.getPose().getY());
        telemetry.addData("Heading", driveSubsystem.getPose().getHeading());


        telemetry.addLine("//Shooter//");
        telemetry.addData("Shooter State", Robot.tuningMode ? "TUNING" : Robot.shooterState.toString());
        telemetry.addLine();

        telemetry.addData("Target Angle", shooterSubsystem.targetPos);
        telemetry.addData("Current Angle", shooterSubsystem.getPosition());
        telemetry.addLine();

        telemetry.addData("Flywheel Velocity", flywheelSubsystem.getVelocity());
        telemetry.addData("Flywheel Target", flywheelSubsystem.lastTargetRadPerSec);
        telemetry.addData("Flywheel Volts", flywheelSubsystem.lastTargetVolts);
        telemetry.addLine();



        telemetry.addLine("//Vision//");
        telemetry.addData("LL Valid", vision.llValid);
        telemetry.addData("Has Tag", vision.hasTag);
        telemetry.addData("Ta", vision.getTa().orElse(-1.0));
        telemetry.addData("Tx", vision.getTx().orElse(-1.0));
        telemetry.addData("Ty", vision.getTy().orElse(-1.0));
        telemetry.addData("Distance", vision.getDistance().orElse(-1.0));
        telemetry.addLine();



        telemetry.update();
    }

}