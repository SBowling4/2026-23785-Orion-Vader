package org.firstinspires.ftc.lib.orion.util;

import org.firstinspires.ftc.lib.wpilib.math.geometry.Translation2d;
import org.firstinspires.ftc.lib.wpilib.math.util.Units;

public class Field {
    public static final double FIELD_LENGTH = Units.inchesToMeters(141.5);

    public static final Translation2d RED_GOAL = new Translation2d(FIELD_LENGTH, FIELD_LENGTH);
    public static final Translation2d BLUE_GOAL = new Translation2d(0, FIELD_LENGTH);

    public static final Translation2d FAR_TARGET_OFFSET = new Translation2d(Units.inchesToMeters(18), 0);
    public static final Translation2d CLOSE_TARGET_OFFSET = new Translation2d(Units.inchesToMeters(4.5), 0);
}
