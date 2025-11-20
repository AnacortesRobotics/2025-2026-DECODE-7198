package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;


public class SecondCompIndexer implements Subsystem {
    private Servo indexServo;
    private Servo loaderServo;
    private String artifactCombination;

    private String location1;
    private String location2;
    private String location3;

    private Telemetry telemetry;
    private CRServo intakeServo;
    private final double POWER = .5;

    public SecondCompIndexer(HardwareMap hMap, Telemetry telemetry) {
        indexServo = hMap.get(Servo.class, "indexServo");
        intakeServo = hMap.get(CRServo.class, "intakeMotor");
        this.telemetry = telemetry;
    }

    public enum sequence{
        PPG,
        PGP,
        GPP
    }

    public Command turnIntake() {
        if (location1 == location2 && location2 == location3){
            return null;//no order
        }

        else if (artifactCombination == "PPG") {
            if (location3 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                                ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        )
                );
            }
            if (location2 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        )
                );
            }
            if (location1 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        )
                );
            }

        }

        else if (artifactCombination == "PGP") {
            if (location3 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        )
                );
            }
            if (location2 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        )
                );
            }
            if (location1 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        )
                );
            }

        }

        else if (artifactCombination == "GPP") {
            if (location3 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        )
                );
            }
            if (location2 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        )
                );
            }
            if (location1 == "green"){
                return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> indexServo.setPosition(0)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(120)
                        ),
                        new WaitCommand(1000),
                        new InstantCommand(
                                () -> indexServo.setPosition(240)
                        )
                );
            }

        }

        return null;

//        return new InstantCommand (
//                () -> setIntakePower(POWER)
//        );
    }

    public void setIntakePower(double power) {
        intakeServo.setPower(power);
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
