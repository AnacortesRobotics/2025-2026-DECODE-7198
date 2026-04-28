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
    private Telemetry telemetry;
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
        this.limelight = hwM.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100);
        this.limelight.pipelineSwitch(1);
        this.limelight.start();
    }

    public void setTarget(VisionTarget target) {
        if (target != this.target) {
            this.target = target;
            switchColorTracking();
            }
    }

    public VisionTarget getTarget() {
        return this.target;
    }

    private void switchColorTracking(){
        if (this.target == VisionTarget.GREEN_ARTIFACT){
            setPipeline(1);
            trackingGreen = true;
        } else if(this.target == VisionTarget.PURPLE_ARTIFACT){
            setPipeline(2);
            trackingGreen = false;
        }
    }

    public void update() {// Automatically fetch new data every loop
        this.result = limelight.getLatestResult();
        this.colorTargets = result.getColorResults();
        this.targetX = this.result.getTx();
        this.targetY = this.result.getTy();
        this.targetArea = this.result.getTa();
        for (LLResultTypes.ColorResult colorTarget : colorTargets) {
            this.colorTargetX = colorTarget.getTargetXDegrees();
            this.colorTargetY = colorTarget.getTargetYDegrees();
            this.colorTargetArea = colorTarget.getTargetArea();
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Pipeline", this.limelight.getStatus().getPipelineIndex());
        if (this.result != null && this.result.isValid()) {
            telemetry.addData("Target X", this.targetX);
            telemetry.addData("Target Y", this.targetY);
            telemetry.addData("Target Area", this.targetArea);
            telemetry.addData("Color Target X", this.colorTargetX);
            telemetry.addData("Color Target Y", this.colorTargetY);
            telemetry.addData("Color Target Area", this.colorTargetArea);
            telemetry.addData("tracking Green ?",trackingGreen);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }

    public void setPipeline(int pipeline) { // when using put in an instant command
        this.limelight.pipelineSwitch(pipeline);
    }        //1 is green and 2 is purple


    public boolean targetDetected() {
        return result != null && result.isValid();
    }

    public double getTargetXOffset() {
        return targetDetected() ? result.getTx() : 0.0;
    }

    public Pose3D getBotPose() {
        return targetDetected() ? this.result.getBotpose() : null;
    }
}
