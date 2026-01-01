package org.firstinspires.ftc.lib.orion.BaseOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.lib.orion.util.Alliance;
import org.firstinspires.ftc.teamcode.Robot;

public class BaseRedOpMode extends OpMode {

    @Override
    public void init() {
        Robot.alliance = Alliance.RED;
        Robot.sendHardwareMap(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
