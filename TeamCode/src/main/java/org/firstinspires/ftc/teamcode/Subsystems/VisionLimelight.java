package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;

import java.util.List;


public class VisionLimelight implements Subsystem {
    public enum VisionTarget {
        GREEN_ARTIFACT,
        PURPLE_ARTIFACT
    }
    private final Limelight3A limelight; // can only declare one limelight
    private final Telemetry telemetry;
    private int pipeline;
    private LLResult result;
    private List<LLResultTypes.ColorResult> colorTargets;
    public double targetX;
    private double targetY;
    private double targetArea;
    public double colorTargetX;
    public double colorTargetY;
    public double colorTargetArea;
    public VisionTarget target;
    public boolean trackingGreen;


    public VisionLimelight(HardwareMap hwM, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hwM.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();
    }

    public void setTarget(VisionTarget target) {
        if (target != this.target) {
            this.target = target;
            switchColorTracking();
            }
    }

    public VisionTarget getTarget() {
        return target;
    }

    private void switchColorTracking(){
        if (target == VisionTarget.GREEN_ARTIFACT){
            setPipeline(1);
            trackingGreen = true;
        } else if(target == VisionTarget.PURPLE_ARTIFACT){
            setPipeline(2);
            trackingGreen = false;
        }
    }

    public void update() {// Automatically fetch new data every loop
        result = limelight.getLatestResult();
        colorTargets = result.getColorResults();
        targetX = -result.getTx();
        targetY = result.getTy();
        targetArea = result.getTa();
        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            colorTargetX = colorTarget.getTargetXDegrees();
            colorTargetY = colorTarget.getTargetYDegrees();
            colorTargetArea = colorTarget.getTargetArea();
        }
    }

    public void updateTelemetry() {
//        telemetry.addData("Pipeline", limelight.getStatus().getPipelineIndex());
        if (result != null && result.isValid()) {
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Area", targetArea);
            telemetry.addData("Color Target X", colorTargetX);
            telemetry.addData("Color Target Y", colorTargetY);
            telemetry.addData("Color Target Area", colorTargetArea);
            telemetry.addData("tracking Green ?",trackingGreen);
        } else {
            telemetry.addLine("Limelight No Targets");
        }
    }

    public void setPipeline(int pipeline) { // when using put in an instant command
        limelight.pipelineSwitch(pipeline);
    }   //1 is green and 2 is purple #3 for pollen


    public boolean targetDetected() {
        return result != null && result.isValid();
    }

    public double getTargetXOffset() {
        return targetDetected() ? result.getTx() : 0.0;
    }

    public Pose3D getBotPose() {
        return targetDetected() ? result.getBotpose() : null;
    }
}
