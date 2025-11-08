package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;


public class SecondCompIndexer implements Subsystem {
    private Servo indexServo;
    private Telemetry telemetry;
    private CRServo intakeMotor;
    private final double POWER = .5;

    public SecondCompIndexer(HardwareMap hMap, Telemetry telemetry) {
        indexServo = hMap.get(Servo.class, "indexServo");
        intakeMotor = hMap.get(CRServo.class, "intakeMotor");
        this.telemetry = telemetry;
    }

    public Command turnIntake() {// run when pressing bumper
        return new InstantCommand (
                () -> setIntakePower(POWER)
        );
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }


    public Command stopIntake() {// run when not pressing bumper
        return new InstantCommand (
                () -> setIntakePower(0)
        );
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
