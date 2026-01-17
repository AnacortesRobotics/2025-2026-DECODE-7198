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

    public static RobotPose FIELD_CENTER = new RobotPose(0, 0, 0);

    /**
     * The target pose for the Red Alliance side of the field.
     * <p>X: 71.0in, Y: -71.0in, Heading: 0.0°
     */
    public static RobotPose RED_TARGET_ANGLE = new RobotPose(71, -71, 0);
    /**
     * The target pose for the Blue Alliance side of the field.
     * <p>X: 71.0in, Y: 71.0in, Heading: 0.0°
     */
    public static RobotPose BLUE_TARGET_ANGLE = new RobotPose(71, 71, 0);

    public static RobotPose BLUE_SETUP_POSE = new RobotPose(-72, 24, 0, true);
    public static RobotPose START_POSE = new RobotPose(-69, 25, 15.75, true);
    public static RobotPose POSE_FAR_SHOOT = new RobotPose(-69, 25, 16.0, true);
    public static RobotPose MOVE_TO_COLLECT_1ST_CYCLE = new RobotPose(-45, 36, 90, true);
    public static RobotPose MOVE_TO_COLLECT_1ST_CYCLE_1ST_BALL = MOVE_TO_COLLECT_1ST_CYCLE.plusY(5);//new RobotPose(-45, 37, 90);
    public static RobotPose MOVE_TO_COLLECT_1ST_CYCLE_2ND_BALL = MOVE_TO_COLLECT_1ST_CYCLE.plusY(10);
    public static RobotPose MOVE_TO_COLLECT_1ST_CYCLE_3RD_BALL = MOVE_TO_COLLECT_1ST_CYCLE.plusY(15);

}