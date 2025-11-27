package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import java.util.List;
import java.util.ArrayList;



public class Limelight implements Subsystem {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Servo pitchServo;
    private boolean isShootingMode;
    private LLResult limelightResult;
    boolean greenTracking;
    Chassis chassis;




    public Limelight(HardwareMap hwM, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hwM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        pitchServo = hwM.get(Servo.class, "limelightServo");

    }

    private LLResultTypes.FiducialResult getAprilTag(boolean isRed) {
        List<LLResultTypes.FiducialResult> result = limelightResult.getFiducialResults();
        if(result == null) return null;
        for (LLResultTypes.FiducialResult tag : result ) {
            if (tag.getFiducialId() == 20 && !isRed) {
                return tag;
            } else if (tag.getFiducialId() == 24 && isRed) {
                return tag;
            }
        }
        return null;
    }

    public double getAngleOffSet(boolean isRed) {
        LLResultTypes.FiducialResult tag = getAprilTag(isRed);
        if(tag == null) return 0.0;
        return (tag.getTargetXDegrees());

    }

    public void update(){
        limelightResult = limelight.getLatestResult();
    }

    public void setServoPosition(double position) {
        pitchServo.setPosition(position);
    }

    public List<Boolean> artifactOrder() {
        LLResultTypes.FiducialResult aprilTag = getAprilTag();
        ArrayList<Boolean> order = new ArrayList<Boolean>();
        if (aprilTag == null) {
            return new ArrayList<>();
        } else if (aprilTag.getFiducialId() == 21) {
            order.add(true);
            order.add(false);
            order.add(false);
        } else if (aprilTag.getFiducialId() == 22) {
            order.add(false);
            order.add(true);
            order.add(false);
        } else if (aprilTag.getFiducialId() == 23) {
            order.add(false);
            order.add(false);
            order.add(true);
        }
        return order;
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

    public void printTelemetry() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
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
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            if (colorResults.size() > 0) {
                return colorResults.get(0);
            }
        }
        return null;
    }

    public Command turnToArtifact() {
        return chassis.autoTurn(()->0, ()->0, getAngleOffSet(true)); //TODO change to isRed

    }

    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);

    }


}
