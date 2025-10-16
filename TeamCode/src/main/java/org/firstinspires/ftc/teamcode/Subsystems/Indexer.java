package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;


public class Indexer implements Subsystem {
    private CRServo indexServo;
    private Telemetry telemetry;

    public void init(HardwareMap hMap, Telemetry telemetry) {
        indexServo = hMap.get(CRServo.class, "indexServo");
        this.telemetry = telemetry;
    }
    private void nextArtifact(){

    }

}
