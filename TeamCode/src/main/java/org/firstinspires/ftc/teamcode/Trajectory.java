package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface Trajectory {

    /* Data that could be good:
    If the point is and end point  2
    A max speed  1
    An approach speed, like a slowdown when close to the point for tight cornering  1
    How close the robot needs to get to the point to move on (reaches look ahead distance, closer, all the way?)  2
    Look ahead distance  2
     */

    public Pose2D getLookaheadPoint(Pose2D currentPose, double lookAheadInches);
    // Make a custom point class to store Pose2D + other data

}
