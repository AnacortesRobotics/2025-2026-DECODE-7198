package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;


import java.util.List;


public class Indexer implements Subsystem {
    private Servo rotationServo;
    private Servo pitchServo;
    private CRServo intakeServo;
//    private DcMotorEx rotationPos;
//    private DigitalChannel magnetSensor;
    private RevColorSensorV3 colorSensor;
//    private Rev2mDistanceSensor distanceSensor;
    private TouchSensor intakeIn;

    private Telemetry telemetry;

    private final double SHOOTING_POS = 180;
    private final double INTAKE_POS = 0;

    private double currentSlot = 0;

    private List<Double> greenSlots;
    private List<Double> purpleSlots;

    private double targetAngle = 0;

    public Indexer(HardwareMap hMap, Telemetry telemetry) {
        rotationServo = hMap.get(Servo.class, "rotationServo");
        pitchServo = hMap.get(Servo.class, "pitchServo");
//        rotationPos = hMap.get(DcMotorEx.class, "rotationPos");
//        magnetSensor = hMap.get(DigitalChannel.class, "magnetSensor");
        colorSensor = hMap.get(RevColorSensorV3.class, "colorSensor");
        intakeServo = hMap.get(CRServo.class, "intakeServo");
        intakeIn = hMap.get(TouchSensor.class, "touch");
//        distanceSensor = hMap.get(Rev2mDistanceSensor.class, "distanceSensor");
//        distanceSensor.initialize();
        this.telemetry = telemetry;
    }

    public void setIntakePower(double power) {
        intakeServo.setPower(power);
    }

    public void setSpindexerPitch(double angle) {
        pitchServo.setPosition(angle);
    }

    public void setSpindexerTarget(double angle, double slot) {
        double target = angle + slot;
        if (target > 360) {
            rotationServo.setPosition(target - 360);
        } else if (target < 0) {
            rotationServo.setPosition(target + 360);
        } else {
            rotationServo.setPosition(target);
        }
    }

//    public double getAngle() {
//        return Math.abs(((double)rotationPos.getCurrentPosition() / 22.75555) % 360);
//    }

//    public void updateAngle() {
//        rotationServo.setPower(isAtTarget() ? 0 : -(getAngle() - targetAngle) > 0 ? .1 : -.1);
//    }

//    public boolean isAtTarget() {
//        double angleDifference = Math.abs(getAngle() - targetAngle);
//        return angleDifference < 4 || angleDifference > 356;
//    }

//    public double getTargetAngle(double offset) {
//        double angle = targetAngle + offset;
//        if (angle > 360) {
//            angle -= 360;
//        } else if (angle < 0) {
//            angle +=360;
//        }
//        return angle;
//    }

    private enum IndexState {
        NOBALLS,
        GREEN,
        PURPLE
    }

    private IndexState getColorResult() {
        if (colorSensor.red() < 100 && colorSensor.green() < 100 && colorSensor.blue() < 100) {
            return IndexState.NOBALLS;
        } else if ((colorSensor.red() + colorSensor.blue()) / 2 > colorSensor.green()) {
            return IndexState.PURPLE;
        } else {
            return IndexState.GREEN;
        }
    }

    public Command startIntake() {
        return new InstantCommand(()->setIntakePower(-1));
    }

    public Command stopIntake() {
        return new InstantCommand(()->setIntakePower(0));
    }

    public Command intakeSlot(double slot) {
        currentSlot = slot;
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerTarget(INTAKE_POS, slot)),
                new WaitCommand(100),
                new InstantCommand(()->setSpindexerPitch(.2))
        ).setInterruptable(true).setName("Intake Slot");
    }

    public Command intakeAndScan() {
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerPitch(.6)),
                new WaitCommand(150),
                new FunctionalCommand(()->{}, ()->{
                    if (getColorResult() != IndexState.NOBALLS) {
                        assignSlot(currentSlot, getColorResult() == IndexState.GREEN);
                    } else {
                        setSpindexerTarget(INTAKE_POS + 5, currentSlot);
                    }},
                    (interrupted)->{},
                    ()->purpleSlots.contains(currentSlot) || greenSlots.contains(currentSlot),
                    this
                )
        ).setInterruptable(true);
    }

//    public Command intakeMode() {
//
//    }

    private void intakeOpen() {
        if (!greenSlots.contains(RobotCoefficients.SLOT1) || !purpleSlots.contains(RobotCoefficients.SLOT1)) {
            intakeSlot(RobotCoefficients.SLOT1);
        } else if ((!greenSlots.contains(RobotCoefficients.SLOT2) || !purpleSlots.contains(RobotCoefficients.SLOT2))) {
            intakeSlot(RobotCoefficients.SLOT2);
        } else if ((!greenSlots.contains(RobotCoefficients.SLOT3) || !purpleSlots.contains(RobotCoefficients.SLOT3))) {
            intakeSlot(RobotCoefficients.SLOT3);
        } else {

        }
    }

    public Command fireSlot(double slot) {
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerTarget(SHOOTING_POS, slot)),
                new WaitCommand(200),
                new InstantCommand(()->setSpindexerPitch(.8)),
                new WaitCommand(200),
                new InstantCommand(()->setSpindexerPitch(.6))
        ).setInterruptable(true).setName("Fire slot");
    }

    private void assignSlot(double slot, boolean isGreen) {
        if (isGreen) {
            greenSlots.add(slot);
        } else {
            purpleSlots.add(slot);
        }
    }

    public void updateTelemetry() {
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
//        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
//        telemetry.addData("is magnet active", !magnetSensor.getState());
//        telemetry.addData("Spindexer position", getAngle());
        telemetry.addData("Spindexer Target", targetAngle);
        telemetry.addData("Is intake up", intakeIn.isPressed());
//        telemetry.addData("Spindexer error", Math.abs(getAngle() - targetAngle));
    }

}
