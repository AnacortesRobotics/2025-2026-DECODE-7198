package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations.AnimationBase;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;



public class Indexer implements Subsystem {
    private Servo rotationServo;
    private Servo pitchServo;
    private DcMotorEx intake;
    private RevColorSensorV3 colorSensor;
    private TouchSensor intakeIn;
//    private GoBildaPrismDriver prism;

    private Telemetry telemetry;
    private TriggerList triggerList;

    private final double SHOOTING_POS = 0;
    private final double INTAKE_POS = 180;

    private double currentSlot = 0;

    private int motifIndex = 0;
    private List<Double> greenSlots = new ArrayList<>();
    private List<Double> purpleSlots = new ArrayList<>();
    private List<Boolean> motifOrder = new ArrayList<>();

    private double targetAngle = 0;

    private BooleanSupplier isIntakeUpIntake;
//    private BooleanSupplier isIntakeUpScan;
    private BooleanSupplier isCurrentSlotFilled;

    public Indexer(HardwareMap hMap, Telemetry telemetry) {
        rotationServo = hMap.get(Servo.class, "rotationServo");
        pitchServo = hMap.get(Servo.class, "pitchServo");
        colorSensor = hMap.get(RevColorSensorV3.class, "colorSensor");
        intake = hMap.get(DcMotorEx.class, "intake");
        intakeIn = hMap.get(TouchSensor.class, "touch");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        prism = hMap.get(GoBildaPrismDriver.class, "prism");
//        prism.setTargetFPS(60);
//        prism.setStripLength(12);
        this.telemetry = telemetry;
        triggerList = TriggerList.getInstance();
        isIntakeUpIntake = this::isIntakeUp;
//        isIntakeUpScan = this::isIntakeUp;
        isCurrentSlotFilled = this::isSlotFull;
        setMotifOrder(true, false, false);
    }

    private boolean isIntakeUp() {
        return intakeIn.isPressed();
    }

    private boolean isSlotFull() {
        return purpleSlots.contains(currentSlot) || greenSlots.contains(currentSlot);
    }

    public void setMotifOrder(Boolean... order) {
        motifOrder = Arrays.asList(order);
    }

    public List<Boolean> getMotifOrder() {
        return motifOrder;
    }

    public void incrementIndex() {
        motifIndex = motifIndex + 1 > 3 ? 0 : motifIndex + 1;
    }

    public void emptySlot(double slot) {
        greenSlots.remove(slot);
        purpleSlots.remove(slot);
//        animateArtifacts(slot, Color.TRANSPARENT);
    }

//    public void animatePrism(LayerHeight height, AnimationBase animation) {
//        prism.insertAndUpdateAnimation(height, animation);
//    }
//
//    public void animateArtifacts(double slot, Color color) {
//        int index = (int)slot / 120;
//        prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, new PrismAnimations.Solid(color, 100, index, index + 1));
//        prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, new PrismAnimations.Solid(color, 100, index + 6, index + 7));
//    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setSpindexerPitch(double angle) {
        pitchServo.setPosition(angle);
    }

    public void setSpindexerTargetInit(double angle) {
        rotationServo.setPosition(angle/360);
    }

    public Command fixIntake() {
        return new InstantCommand(()->setSpindexerTargetInit(rotationServo.getPosition() + 30));
    }


    public void setSpindexerTarget(double angle, double slot) {
        double target = angle + slot;
        currentSlot = slot;
        if (target > 360) {
            rotationServo.setPosition((target - 360)/360);
        } else if (target < 0) {
            rotationServo.setPosition((target + 360)/360);
        } else {
            rotationServo.setPosition(target/360);
        }
    }

    private enum IndexState {
        NOBALLS,
        GREEN,
        PURPLE
    }

    private IndexState getColorResult() {
        if (colorSensor.green() < 65 && colorSensor.blue() < 65) {
            return IndexState.NOBALLS;
        } else if (colorSensor.blue() > colorSensor.green()) {
            return IndexState.PURPLE;
        } else {
            return IndexState.GREEN;
        }
    }

    public Command startIntake() {
        return new InstantCommand(()->setIntakePower(.7));
    }

    public Command reverseIntake() {
        return new InstantCommand(()->setIntakePower(-.7));
    }

    public Command stopIntake() {
        return new InstantCommand(()->setIntakePower(0));
    }

    public Command intakeSlot(double slot) {
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerTarget(INTAKE_POS, slot)),
                new WaitCommand(1000),
                startIntake(),
                new InstantCommand(()->setSpindexerPitch(.3))
        ).setInterruptable(true).setName("Intake Slot");
    }

    public Command intakeAndScan() {
        triggerList.removeTrigger(isIntakeUpIntake);
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerPitch(.58)),
                new WaitCommand(750),
                stopIntake(),
                new FunctionalCommand(()->{}, ()->{
                    if (getColorResult() != IndexState.NOBALLS) {
                        assignSlot(currentSlot, getColorResult() == IndexState.GREEN);
                    } else {
                        setSpindexerTarget(INTAKE_POS + 5, currentSlot);
                    }},
                    (interrupted)->{},
                    ()->purpleSlots.contains(currentSlot) || greenSlots.contains(currentSlot),
                    this
                ),
                new InstantCommand(()->setSpindexerPitch(.55))
        ).setInterruptable(true).setName("Intake and scan");
    }

    public Command intakeMode() {
        return new InstantCommand(()-> {
//            triggerList.removeTrigger(isIntakeUpScan);
            triggerList.addTrigger(isIntakeUpIntake).onPressed(intakeAndScan());
            triggerList.addTrigger(isCurrentSlotFilled).onJustPressed(intakeOpen());
        });
    }

    public Command intakeOpen() {
        if (!greenSlots.contains(RobotCoefficients.SLOT1) || !purpleSlots.contains(RobotCoefficients.SLOT1)) {
            return intakeSlot(RobotCoefficients.SLOT1);
        } else if ((!greenSlots.contains(RobotCoefficients.SLOT2) || !purpleSlots.contains(RobotCoefficients.SLOT2))) {
            return intakeSlot(RobotCoefficients.SLOT2);
        } else if ((!greenSlots.contains(RobotCoefficients.SLOT3) || !purpleSlots.contains(RobotCoefficients.SLOT3))) {
            return intakeSlot(RobotCoefficients.SLOT3);
        } else {
            return shootingMode();
        }
    }

    public Command shootingMode() {
        return new InstantCommand(()->{
            triggerList.removeTrigger(isIntakeUpIntake);
            triggerList.removeTrigger(isCurrentSlotFilled);
//            triggerList.removeTrigger(isIntakeUpScan);
        });
    }

    public Command fireInOrder() {
        if (purpleSlots.isEmpty() && greenSlots.isEmpty()) {
            return intakeMode();
        } else if (motifOrder.get(motifIndex) == true && !greenSlots.isEmpty()) {
            return fireSlot(greenSlots.get(0));
        } else if (motifOrder.get(motifIndex) == false && !purpleSlots.isEmpty()) {
            return fireSlot(purpleSlots.get(0));
        } else {
            return fireSlot(greenSlots.isEmpty() ? purpleSlots.get(0) : greenSlots.get(0));
        }
    }

    public Command fireSlot(double slot) {
        double lastCurrentSlot = currentSlot;
        return new SequentialCommandGroup(
                new InstantCommand(()->setSpindexerPitch(.55)),
                new InstantCommand(()->setSpindexerTarget(SHOOTING_POS, slot)),
                new WaitCommand(()->{
                    if (lastCurrentSlot == slot) {
                        return 0;
                    } else if (Math.abs(lastCurrentSlot - slot) == 120) {
                        return 300;
                    } else {
                        return 500;
                    }
                })

        ).setInterruptable(true).setName("Fire slot");
    }

    public Command pitchToLauncher() {
        return new SequentialCommandGroup(
                new InstantCommand(()->{setSpindexerPitch(.65);}),
                new WaitCommand(100),
                new InstantCommand(()->setSpindexerPitch(.55))
        );
    }

    private void assignSlot(double slot, boolean isGreen) {
        if (isGreen) {
            greenSlots.add(slot);
//            animateArtifacts(slot, Color.GREEN);
        } else {
            purpleSlots.add(slot);
//            animateArtifacts(slot, Color.PURPLE);
        }
    }

    public void updateTelemetry() {
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("Spindexer Target", targetAngle);
        telemetry.addData("Is intake up", intakeIn.isPressed());
        telemetry.addData("Spindexer Pos", rotationServo.getPosition());
        telemetry.addData("Green list", greenSlots.toString());
        telemetry.addData("Purple list", purpleSlots.toString());
    }

}
