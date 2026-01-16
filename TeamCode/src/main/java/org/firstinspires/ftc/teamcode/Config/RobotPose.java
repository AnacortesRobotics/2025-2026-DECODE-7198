package org.firstinspires.ftc.teamcode.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A specialized version of {@link Pose2D} tailored for FTC field navigation.
 * <p>This class removes 1/2 of chassis length and width so that field position is based
 *  from center of robot. It also defaults to Inches and Degrees to reduce "parameter fatigue" and
 * stores coordinates locally to avoid repeated unit conversion math during high-frequency
 * control loops. </p>
 */
public class RobotPose extends Pose2D {

    private final double xInches;
    private final double yInches;
    private final double headingDegrees;

    /**
     * Fast Constructor: Assumes coordinates are based on field only.
     * Constructs a RobotPose with predefined units.
     * @param x       The X coordinate of the field.
     * @param y       The Y coordinate of the field.
     * @param heading The heading in degrees.
     */
    public RobotPose(double x, double y, double heading) {
        // Direct assignment to super - Fastest execution path
        super(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        this.xInches = x;
        this.yInches = y;
        this.headingDegrees = heading;
    }

    /**
     * Logic Constructor: Allows defining a pose relative to the center of the robot.
     * Constructs a RobotPose with predefined units. It always subtracts half of chassis length and width
     * @param x                      The coordinate (either center or wall).
     * @param y                      The coordinate (either center or wall).
     * @param heading                The heading in degrees.
     * @param isRobotCenterReference If True, shifts inputs inward by Robot Half-Width/Length.
     * If false, x/y are used as-is.
     */
    public RobotPose(double x, double y, double heading, boolean isRobotCenterReference) {
        super(
                DistanceUnit.INCH,
                // If it IS the center, subtract the shift, else use incoming parameters x and y.
                isRobotCenterReference ? x - Math.copySign(RobotCoefficients.ROBOT_LENGTH_HALF, x) : x,
                isRobotCenterReference ? y - Math.copySign(RobotCoefficients.ROBOT_WIDTH_HALF, y) : y,
                AngleUnit.DEGREES,
                heading
        );

        this.xInches = super.getX(DistanceUnit.INCH);
        this.yInches = super.getY(DistanceUnit.INCH);
        this.headingDegrees = heading;
    }

    /**
     * Creates a new RobotPose shifted by X and Y distances without changing the heading.
     *
     * @param deltaX The distance to add to the X coordinate (inches).
     * @param deltaY The distance to add to the Y coordinate (inches).
     * @return A new RobotPose object with the same heading.
     */
    public RobotPose offset(double deltaX, double deltaY) {
        return offset(deltaX, deltaY, 0);
    }

    /**
     * Creates a new RobotPose shifted by X and Y distances and a heading offset.
     * <p>This is the primary logic method; all other offsets call this one.</p>
     *
     * @param deltaX       The distance to add to the X coordinate (inches).
     * @param deltaY       The distance to add to the Y coordinate (inches).
     * @param deltaHeading The angle to add to the current heading (degrees).
     * @return A new RobotPose object with updated position and rotation.
     */
    public RobotPose offset(double deltaX, double deltaY, double deltaHeading) {
        return new RobotPose(
                this.xInches + deltaX,
                this.yInches + deltaY,
                this.headingDegrees + deltaHeading
        );
    }

    /**
     * Creates a new RobotPose with a shift applied only to the X axis.
     *
     * @param deltaX The distance to add to the X coordinate (inches).
     * @return A new RobotPose object.
     */
    public RobotPose plusX(double deltaX) {
        return offset(deltaX, 0, 0);
    }

    /**
     * Creates a new RobotPose with a shift applied only to the Y axis.
     *
     * @param deltaY The distance to add to the Y coordinate (inches).
     * @return A new RobotPose object.
     */
    public RobotPose plusY(double deltaY) {
        return offset(0, deltaY, 0);
    }

    /**
     * Creates a new RobotPose with a rotation offset applied to the heading.
     *
     * @param deltaHeading The angle to add to the current heading (degrees).
     * @return A new RobotPose object.
     */
    public RobotPose plusHeading(double deltaHeading) {
        return offset(0, 0, deltaHeading);
    }

    /**
     * Returns the X coordinate in inches.
     *
     * @return X in inches.
     */
    public double x() {
        return xInches;
    }

    /**
     * Returns the Y coordinate in inches.
     *
     * @return Y in inches.
     */
    public double y() {
        return yInches;
    }

    /**
     * Returns the heading in degrees.
     *
     * @return Heading in degrees.
     */
    public double h() {
        return headingDegrees;
    }
}