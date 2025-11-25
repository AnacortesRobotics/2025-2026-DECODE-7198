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

import java.util.List;


public class LimelightPurpleArtifact implements Subsystem {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Servo pitchServo;
    private boolean isShootingMode;
    private LLResult limelightResult;


    public LimelightPurpleArtifact(HardwareMap hwM, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hwM.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
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

    public Command shootingMode() {
        if(isShootingMode) {
            return new InstantCommand(
                    () -> pitchServo.setPosition(.5)
            );
        }
        if(!isShootingMode) {
            return new InstantCommand(
                    () -> pitchServo.setPosition(0)
            );

        }
        else return null;
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

    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);

    }


}
