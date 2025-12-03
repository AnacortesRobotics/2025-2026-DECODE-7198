package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

import java.util.List;

public class ValueTurnover {

    private static ValueTurnover instance;

    private static Pose2D currentPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private static boolean isRed = false;
    private static List<Boolean> motif;


    public static synchronized ValueTurnover getInstance() {
        if (instance == null) {instance = new ValueTurnover();}
        return instance;
    }

    public void setCurrentPos(Pose2D currentPos) {
        currentPose = currentPos;
    }

    public Pose2D getCurrentPos() {
        return currentPose;
    }

    public void setIsRed(boolean isRedSide) {
        isRed = isRedSide;
    }

    public boolean getIsRed() {
        return isRed;
    }

}
