package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Commands.*;
import java.util.List;
import java.util.ArrayList;



public class Limelight implements Subsystem {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Servo pitchServo;
    private boolean isShootingMode;
    private LLResult result;
    private List<LLResultTypes.ColorResult> colorTargets;
    private double targetX;
    private double targetY;
    boolean greenTracking;
    Chassis chassis;

    public Limelight(HardwareMap hwM, Telemetry telemetry, Chassis chassis) {
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
        LLResultTypes.ColorResult tag = getColorTrackingResults();
        if(tag == null && !result.isValid()) return 0.0;
        return -tag.getTargetXDegrees();
    }

    public void update(){
        result = limelight.getLatestResult();
        colorTargets = result.getColorResults();
        updateTelemetry();
    }
    public void updateTelemetry(){
        if (result != null && result.isValid()) {//check if it sees something
            targetX = result.getTx(); // How far left or right the target is (degrees)
            targetY = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Area", ta);
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
            else return null;//colorTargets.get(0);
        }
        return null;
    }

    public Command turnToArtifact() {
        telemetry.addData("angle offset", chassis.getPose().getHeading(AngleUnit.DEGREES));
        return chassis.autoTurn(
                    () -> 0, () -> 0,
                    /*chassis.getPose().getHeading(AngleUnit.DEGREES) +*/ targetX).setInterruptable(true);
    }

    public Command setPipeline(int pipeline){
        return new InstantCommand(()->limelight.pipelineSwitch(pipeline));
        //1 is green and 2 is purple
    }


}
