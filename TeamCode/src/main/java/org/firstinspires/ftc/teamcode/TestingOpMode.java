package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp
public class TestingOpMode extends OpMode {

    //private AnalogInput encoder;
    private GoBildaPrismDriver prism;

    private List<Double> green = new ArrayList<>();
    private List<Double> purple = new ArrayList<>();


//    private DistanceSensor distance;
//    private DigitalChannel touchSensor;
    private CommandScheduler commandScheduler;
//    static final double MAX_POS = 1.0;
//    static final double MIN_POS = 0.0;
//    double  position = (MAX_POS - MIN_POS) / 2;
//    Servo tServo;
//    CRServo sServo;
//    NormalizedColorSensor colorSensor;


    @Override
    public void init() {

//        encoder = hardwareMap.get(AnalogInput.class, "indexerPOS");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);
        prism.clearAllAnimations();
        prism.setStripLength(12);
        prism.setTargetFPS(60);
        PrismAnimations.Blink anim1 = new PrismAnimations.Blink();
        anim1.setPrimaryColor(Color.RED);
        anim1.setSecondaryColor(Color.TRANSPARENT);
        anim1.setBrightness(100);
        anim1.setPeriod(500, TimeUnit.MILLISECONDS);
//        PrismAnimations.DroidScan anim2 = new PrismAnimations.DroidScan(Color.PURPLE);
//        anim2.setPrimaryColor(Color.PURPLE);
//        anim2.setSecondaryColor(Color.RED);
//        anim2.setEyeWidth(1);
//        anim2.setTrailWidth(1);
//        anim2.setBrightness(90);
//        anim2.setSpeed(.1f);
//        anim2.setStartIndex(0);
//        anim2.setStopIndex(11);
//        anim2.setDroidScanStyle(PrismAnimations.DroidScan.DroidScanStyle.BACK_TAIL);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_9, anim1);
//        prism.insertAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, anim2);
//        prism.setDefaultBootArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
//        prism.enableDefaultBootArtboard(false);

        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{green.add(0.0);
                prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, new PrismAnimations.Solid(Color.GREEN, 0, 1));}
        ));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{green.add(120.0);
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, new PrismAnimations.Solid(Color.GREEN, 2, 3));}
        ));
        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{green.add(240.0);
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_4, new PrismAnimations.Solid(Color.GREEN, 4, 5));}
        ));
        commandScheduler.getTrigger(GamepadInput.DPAD_DOWN, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{purple.add(0.0);
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_2, new PrismAnimations.Solid(new Color(160,0,255), 0, 1));}
        ));
        commandScheduler.getTrigger(GamepadInput.DPAD_RIGHT, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{purple.add(120.0);
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_3, new PrismAnimations.Solid(new Color(160,0,255), 2, 3));}
        ));
        commandScheduler.getTrigger(GamepadInput.DPAD_LEFT, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()->{purple.add(240.0);
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_4, new PrismAnimations.Solid(new Color(160,0,255), 4, 5));}
        ));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(
                ()-> {
                    purple.add(240.0);
                    prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_7);
                }));


//
//        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSens");
//
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
//
//
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSens");
//        distance = hardwareMap.get(DistanceSensor.class, "distanceSens");
//        tServo = hardwareMap.get(Servo.class, "testServo");
//        sServo = hardwareMap.get(CRServo.class, "crServo");

    }

    @Override
    public void loop() {
        prism.updateAllAnimations();
        telemetry.addData("Number of leds", prism.getNumberOfLEDs());
        prism.saveCurrentAnimationsToArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_7);

//        telemetry.addData("pos", encoder.getVoltage() * (360 / encoder.getMaxVoltage()));
//
//        if (!touchSensor.getState()) {
//            telemetry.addData("Button", "PRESSED");
//        } else {
//            telemetry.addData("Button", "NOT PRESSED");
//        }
//        if (gamepad1.aWasPressed()){
//            sServo.setPower(.5);
//        }
//        if (gamepad1.bWasPressed()){
//            sServo.setPower(-.5);
//        }

//        final float[] hsvValues = new float[3];
//
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        if (colors.green>0.015 && colors.green> colors.blue){
//            sServo.setPower((colors.green - colors.blue)*10);
//        }
//        else if(colors.blue>0.015 && colors.blue> colors.green){
//            sServo.setPower(-(colors.blue - colors.green)*10);
//        }
//        else {
//            sServo.setPower(0);
//        }
//
//        telemetry.addLine()
//                .addData("Red", "%.3f", colors.red)
//                .addData("Green", "%.3f", colors.green)
//                .addData("Blue", "%.3f", colors.blue);
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
//
//        telemetry.addData("servo position", tServo.getPosition());
//        telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
        commandScheduler.run();
//        commandScheduler.updateTelemetry();


    }
}
