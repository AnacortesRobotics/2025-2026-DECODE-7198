package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.*;
import java.util.List;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import static java.lang.Runtime.getRuntime;


public class Limelight implements Subsystem {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Servo pitchServo;
    private boolean isShootingMode;
    private LLResult result;
    private List<LLResultTypes.ColorResult> colorTargets;
    private double targetX;
    private double targetY;
    private double targetArea;
    public double colorTargetX;
    public double colorTargetY;
    public double colorTargetArea;
    boolean greenTracking;
    Chassis chassis;
    TriggerList triggerList;

    // ---------------------- turning variables trial -----------------------------
    private double kP = 0.002;
    private double error = 0;
    private double lastError = 0;
    private double goalX = 0; //offset
    private double angleTolerance = 0.4;
    private double kD = 0.0001;
    private double curTime = 0;
    private double lastTime = 0;
    private double[] stepSizes = {1.0, 0.1, 0.001, 0.0001};
    private ElapsedTime runtime;

    public Limelight(HardwareMap hwM, Telemetry telemetry, Chassis chassis, TriggerList triggerList) {
        runtime = new ElapsedTime();
        runtime.reset();
        this.chassis = chassis;
        this.telemetry = telemetry;
        limelight = hwM.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
//        pitchServo = hwM.get(Servo.class, "limelightServo");
    }

    private LLResultTypes.FiducialResult getAprilTag(int aprilTag) {
        List<LLResultTypes.FiducialResult> lLresult = result.getFiducialResults();
        if(lLresult == null) return null;
        for (LLResultTypes.FiducialResult tag : lLresult ) {
            if (tag.getFiducialId() == aprilTag) {
                return tag;
            } else if (tag.getFiducialId() == aprilTag) {
                return tag;
            }
        }
        return null;
    }

    public double getAngleOffSet() {
        if (chassis.getPose().getHeading(AngleUnit.DEGREES) < 0){
            return chassis.getPose().getHeading(AngleUnit.DEGREES) + colorTargetX;
        } else return chassis.getPose().getHeading(AngleUnit.DEGREES) - Math.abs(colorTargetX);
    }

    public void update(){
        result = limelight.getLatestResult();
        colorTargets = result.getColorResults();
        targetX = 50;//-result.getTx();
        targetY = 50;//-result.getTy();
        targetArea = result.getTa();
        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            colorTargetX = colorTarget.getTargetXDegrees();
            colorTargetY = colorTarget.getTargetYDegrees();
            colorTargetArea = colorTarget.getTargetArea();
        }
    }

    public void updateTelemetry(){
        if (result != null && result.isValid()) {
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Area", targetArea);
            telemetry.addData("Color Target X", colorTargetX);
            telemetry.addData("Color Target Y", colorTargetY);
            telemetry.addData("Color Target Area", colorTargetArea);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    public void setServoPosition(double position) {
        pitchServo.setPosition(position);
    }



    public void shootingMode() {
        if(isShootingMode) {
            setPipeline(0);
            pitchServo.setPosition(0);
        }
        if(!isShootingMode) {
            pitchServo.setPosition(1);
            if (greenTracking) {
                setPipeline(1);
                greenTracking = true;
            }
            else if (!greenTracking) {
                setPipeline(2);
                greenTracking = false;
            }
        }
    }

    public void updateShootingMode(boolean shootingMode) {
        isShootingMode = shootingMode;
        shootingMode();
    }

    public LLResultTypes.FiducialResult getAprilTag(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.size() > 0){
            return fiducialResults.get(0);
        } else {
            return null;
        }
    }

    public LLResultTypes.ColorResult getColorTrackingResults() {
        if (result.isValid() && result != null) {
            if (colorTargets.size() > 0) {
                return colorTargets.get(0);
            }
            else return null;
        }
        return null;
    }

//    public Command turnToArtifact() {
//        BooleanSupplier bbb = () -> true;
//
//        return new InstantCommand(()->
//                    chassis.getPose().getHeading(AngleUnit.DEGREES) + targetX)''
//        );
//    }

    public void turnToTest() {
        if (result != null){
            error = goalX - targetX;
            if (Math.abs(error) < angleTolerance){
                //rotate = 0;
            }else {
                double pTerm = error + kP;
//                curTime = getRuntime();
                double dT = curTime - lastTime;
                double dTerm = ((error - lastError)/dT) * kD;
//                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                lastError = error;
                lastTime = curTime;
            }
        } else {
//            lastTime = getRuntime();
            lastError = 0;
        }
    }

    public Command setPipeline(int pipeline){
        return new InstantCommand(()->limelight.pipelineSwitch(pipeline));
        //1 is green and 2 is purple
    }
}
