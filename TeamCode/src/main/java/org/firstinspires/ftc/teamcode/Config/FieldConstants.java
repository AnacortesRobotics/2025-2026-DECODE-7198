package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Global constants for field coordinates and robot positions.
 *
 * <p>This class is annotated with {@code @Config}, allowing values to be modified
 * in real-time via the Dashboard. All positions use the {@link RobotPose} wrapper
 * which defaults to Inches and Degrees.
 */
@Config
public class FieldConstants {
    /** Private constructor to prevent anyone from creating an instance of this class. */
    private FieldConstants() {}

    public static final RobotPose FIELD_CENTER = new RobotPose(0, 0, 0);

    /**
     * The target pose for the Red Alliance side of the field.
     * <p>X: 71.0in, Y: -71.0in, Heading: 0.0°
     */
    public static final RobotPose RED_TARGET_ANGLE = new RobotPose(71, -71, 0);
    /**
     * The target pose for the Blue Alliance side of the field.
     * <p>X: 71.0in, Y: 71.0in, Heading: 0.0°
     */
    public static final RobotPose BLUE_TARGET_ANGLE = new RobotPose(71, 71, 0);

}