package org.firstinspires.ftc.lib.orion.util;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(144);

    public static final Translation2d RED_GOAL = new Translation2d(Units.inchesToMeters(-72),Units.inchesToMeters(72));
    public static final Translation2d BLUE_GOAL = new Translation2d(Units.inchesToMeters(-72), Units.inchesToMeters(-72));
}
