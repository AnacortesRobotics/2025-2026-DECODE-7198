package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;



public class Indexer implements Subsystem {
    private Servo indexServo;
    private Telemetry telemetry;

    public Indexer(HardwareMap hMap, Telemetry telemetry) {
        indexServo = hMap.get(Servo.class, "indexServo");
        this.telemetry = telemetry;
    }

    public Command fireBall() {
        return new SequentialCommandGroup(
                new InstantCommand(this::loadBall, this),
                new WaitCommand(200),
                new InstantCommand(this::retractServo, this));
    }

    private void loadBall() {
        indexServo.setPosition(0);
    }

    private void retractServo() {
        indexServo.setPosition(.7);
    }

    private void nextArtifact() {

    }



}
