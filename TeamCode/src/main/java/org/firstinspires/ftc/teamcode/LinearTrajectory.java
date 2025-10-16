package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Arrays;
import java.util.stream.Stream;

public class LinearTrajectory {

    private Pose2D[] trajectoryPoints;

    private int pointIndex = 0;
    private Pose2D currentPoint;
    private Pose2D targetPoint;

    private double currentX = 0;
    private double currentY = 0;
    private double targetX = 0;
    private double targetY = 0;

    private Telemetry telemetry;

    public LinearTrajectory(Telemetry telemetry, Pose2D... points) {
        queueTargetPoints(points);
        this.telemetry = telemetry;

        if (trajectoryPoints.length == 1) {
            targetPoint = trajectoryPoints[pointIndex];
            currentX = 0;
            currentY = 0;
            targetX = targetPoint.getX(DistanceUnit.INCH);
            targetY = targetPoint.getY(DistanceUnit.INCH);
        } else if (trajectoryPoints.length >= 2) {
            currentPoint = trajectoryPoints[pointIndex];
            targetPoint = trajectoryPoints[pointIndex + 1];
            currentX = currentPoint.getX(DistanceUnit.INCH);
            currentY = currentPoint.getY(DistanceUnit.INCH);
            targetX = targetPoint.getX(DistanceUnit.INCH);
            targetY = targetPoint.getY(DistanceUnit.INCH);
        }

    }

    public Pose2D getLookaheadPoint(Pose2D currentPose, double lookAheadInches) {
        if (trajectoryPoints.length <= 1) {
            return targetPoint;
        }

        double currentPoseX = currentPose.getX(DistanceUnit.INCH);
        double currentPoseY = currentPose.getY(DistanceUnit.INCH);

        double x1 = currentX - currentPoseX;
        double x2 = targetX - currentPoseX;
        double y1 = currentY - currentPoseY;
        double y2 = targetY - currentPoseY;

        double distX = x2 - x1;
        double distY = y2 - y1;
        double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
        double det = x1 * y2 - x2 * y1;
        double discriminant = Math.pow(lookAheadInches, 2) * Math.pow(dist, 2) - Math.pow(det, 2);

        if (discriminant <= 0) {
            return getNearestPointOnLine(currentPose);
        }

        // If this crashed, make sure that you don't have 2 of the same point somehow
        // if dist is zero, the program will crash somewhere

        double x1Intercept = (det * distY + (distY < 0 ? -1 : 1) * distX *
                Math.sqrt(discriminant)) / Math.pow(dist, 2);
        double x2Intercept = (det * distY - (distY < 0 ? -1 : 1) * distX *
                Math.sqrt(discriminant)) / Math.pow(dist, 2);
        double y1Intercept = (-det * distX + Math.abs(distY) * Math.sqrt(discriminant)) / Math.pow(dist, 2);
        double y2Intercept = (-det * distX - Math.abs(distY) * Math.sqrt(discriminant)) / Math.pow(dist, 2);

        double intercept1Dist = Math.sqrt(Math.pow((x2 - x1Intercept), 2) +
                Math.pow(y2 - y1Intercept, 2));

        double intercept2Dist = Math.sqrt(Math.pow((x2 - x2Intercept), 2) +
                Math.pow(y2 - y2Intercept, 2));

        double closestDist = Math.min(intercept1Dist, intercept2Dist);
        double closestX = intercept1Dist < intercept2Dist ? x1Intercept : x2Intercept;
        double closestY = intercept1Dist < intercept2Dist ? y1Intercept : y2Intercept;

        Pose2D lookaheadPoint = new Pose2D(DistanceUnit.INCH, closestX + currentPoseX,
                closestY + currentPoseY,
                AngleUnit.DEGREES,
                lerp(currentPoint.getHeading(AngleUnit.DEGREES), targetPoint.getHeading(AngleUnit.DEGREES),
                        Math.max(Math.min(1 - closestDist / dist, 1), 0)));

        if (isPointOffSegment(lookaheadPoint)) {
            if (isLastPoint()) {
                return targetPoint;
            } else {
                incrementTargetPoints();
                return getLookaheadPoint(currentPose, lookAheadInches);
            }
        } else {
            return lookaheadPoint;
        }

    }

    private boolean isPointOffSegment(Pose2D lookAheadPoint) {
        double lookaheadX = lookAheadPoint.getX(DistanceUnit.INCH);
        double lookaheadY = lookAheadPoint.getY(DistanceUnit.INCH);


        boolean ignoreX = targetX - currentX == 0;
        boolean ignoreY = targetY - currentY == 0;

        boolean isXIncreasing = targetX - currentX > 0;
        boolean isYIncreasing = targetY - currentY > 0;

        return ((isXIncreasing && (targetX < lookaheadX)) ||
                (!isXIncreasing && (targetX > lookaheadX)) || ignoreX)
                && ((isYIncreasing && (targetY < lookaheadY))
                || (!isYIncreasing && (targetY > lookaheadY)) || ignoreY);
    }

    private void incrementTargetPoints() {
        if (trajectoryPoints.length == pointIndex + 2) {
            return;
        }
        pointIndex += 1;

        currentPoint = trajectoryPoints[pointIndex];
        targetPoint = trajectoryPoints[pointIndex + 1];

        currentX = currentPoint.getX(DistanceUnit.INCH);
        currentY = currentPoint.getY(DistanceUnit.INCH);
        targetX = targetPoint.getX(DistanceUnit.INCH);
        targetY = targetPoint.getY(DistanceUnit.INCH);
    }

    private boolean isLastPoint() {
        return pointIndex + 2 == trajectoryPoints.length;
    }

    private Pose2D getNearestPointOnLine(Pose2D currentPose) {
        double currentPoseX = currentPose.getX(DistanceUnit.INCH);
        double currentPoseY = currentPose.getX(DistanceUnit.INCH);

        double lineDistX = targetX - currentX;
        double lineDistY = targetY - currentY;

        double lineSlope = lineDistY / lineDistX;

        double perpLineSlope = -1 / lineSlope;

        double interceptPointX = (currentPoseY - currentY + (lineSlope * currentX) - (perpLineSlope * currentPoseX))
                / (lineSlope - perpLineSlope);
        double interceptPointY = perpLineSlope * (interceptPointX - currentPoseX) + currentPoseY;

        return new Pose2D(DistanceUnit.INCH, interceptPointX, interceptPointY, AngleUnit.DEGREES, targetPoint.getHeading(AngleUnit.DEGREES));
    }

    private void queueTargetPoints(Pose2D... points) {
        trajectoryPoints = points;
        pointIndex = 0;
    }

    private double lerp(double start, double end, double percent) {
        return start + (end - start) * percent;
    }

    private String pathToString() {
        StringBuilder pathString = new StringBuilder();
        for (Pose2D point : trajectoryPoints) {
            pathString.append(point.getX(DistanceUnit.INCH)).append(", ");
            pathString.append(point.getY(DistanceUnit.INCH)).append(", ");
            pathString.append(point.getHeading(AngleUnit.DEGREES)).append(";  ");
        }
        return pathString.toString();
    }

    public void updateTelemetry() {
        telemetry.addData("Point index", pointIndex);
        telemetry.addData("Trajectory points", pathToString());
    }

}
