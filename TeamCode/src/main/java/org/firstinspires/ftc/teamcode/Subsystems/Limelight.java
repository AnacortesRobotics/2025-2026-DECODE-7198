package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;

public class Limelight {

    private Limelight3A limelight;
    private Servo limelightAngle;

    private Telemetry telemetry;

    public Limelight(HardwareMap hMap, Telemetry telemetry) {
        limelight = hMap.get(Limelight3A.class, "limelight");
        limelightAngle = hMap.get(Servo.class, "limelightAngle");
        this.telemetry = telemetry;
    }

    public void switchPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public LLResult getResults() {
        return limelight.getLatestResult();
    }

    public List<Boolean> getMotifOrder(int id) {
        List<Boolean> order = new ArrayList<>();
        if (id == 21) {
            order.add(true);
            order.add(false);
            order.add(false);
        } else if (id == 22) {
            order.add(false);
            order.add(true);
            order.add(false);
        } else if (id == 23) {
            order.add(false);
            order.add(false);
            order.add(true);
        }
        return order;
    }




}
