package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private Telemetry telemetry;

    public Limelight(HardwareMap hwM, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hwM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
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
    /*
    public LLResultTypes.FiducialResult getAprilTag(){
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults.size() > 0){
            return fiducialResults.get(0);
        } else {
            return null;
        }
    }
    */
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

    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);

    }


}
