package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public enum DefinedPose {
    FRONT_START(new Pose(19.090252707581236, 121.15523465703974,Units.degreesToRadians(144))),
    FRONT_ONLY_OFFLINE(new Pose(45.602888086642615, 128.4332129963899, Units.degreesToRadians(180))),
    FRONT_SHOOT(new Pose(52.8808664259928, 85.28519855595668, Units.degreesToRadians(144))),
    GATE_OFFLINE(new Pose(31.873646209386283, 69.68953068592057, Units.degreesToRadians(180))),
    FIRST_PICKUP(new Pose(19.54570397111913, 82.07220216606497, Units.degreesToRadians(180))),
    SECOND_PICKUP(new Pose(14.584837545126353, 59.17328519855596, Units.degreesToRadians(180))),
    OFFLINE_SHOOT(new Pose(56.483754512635386, 114.32490974729241, Units.degreesToRadians(145))),
    SECOND_SHOOT_CONTROL(new Pose(59.51083032490974, 57.13357400722024, Units.degreesToRadians(180))),
    BACK_START(new Pose(57.21299638989169, 9.212996389891728, Units.degreesToRadians(90))),
    BACK_OFFLINE(new Pose(27.685920577617324, 9.234657039711184, Units.degreesToRadians(180))),
    BACK_SHOOT(new Pose(55.93862815884477, 16.104693140794236, Units.degreesToRadians(115))),
    READY_THIRD_PICKUP(new Pose(44.67509025270758, 34.64620938628161, Units.degreesToRadians(180))),
    THIRD_PICKUP(new Pose(15.04332129963898, 36.37906137184118, Units.degreesToRadians(180))),
    SECOND_GATE(new Pose(16.693140794223826, 68.25270758122744, Units.degreesToRadians(90))),
    FIRST_GATE(new Pose(16.693140794223826, 70.25270758122744, Units.degreesToRadians(90))),
    FIRST_PICKUP_GATE_CONTROL(new Pose(37.8754512635379, 78.27075812274369, Units.degreesToRadians(180))),
    SECOND_PICKUP_GATE_CONTROL(new Pose(30.680505415162454, 62.9927797833935, Units.degreesToRadians(180))),
    SECOND_PICKUP_CONTROL(new Pose(68.14440433212997, 52.87703971119132, Units.degreesToRadians(180))),
    THIRD_PICKUP_FRONT_CONTROL(new Pose(72.82310469314079, 30.523249097472913,Units.degreesToRadians(180))),
    SECOND_GATE_TO_SHOOT_CONTROL(new Pose(52.0361010830325, 63.37545126353788, Units.degreesToRadians(144))),
    THIRD_PICKUP_BACK_CONTROL(new Pose(56.31407942238267, 40.57039711191336, Units.degreesToRadians(180))),
    HP_PICKUP(new Pose(11.187725631768954, 9.487364620938633, Units.degreesToRadians(180))),
    RED_FIRST_PICKUP(FIRST_PICKUP.pose.withHeading(190));


    public final Pose pose;
    private DefinedPose(Pose pose) {
        this.pose = pose;
    }
}