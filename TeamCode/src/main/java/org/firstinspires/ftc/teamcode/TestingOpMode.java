package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
//import android.hardware.Sensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

//@Disabled
@TeleOp
public class TestingOpMode extends OpMode {

    AnalogInput encoder;

//    private DistanceSensor distance;
//    private DigitalChannel touchSensor;
//    private CommandScheduler commandScheduler;
//    static final double MAX_POS = 1.0;
//    static final double MIN_POS = 0.0;
//    double  position = (MAX_POS - MIN_POS) / 2;
//    Servo tServo;
//    CRServo sServo;
//    NormalizedColorSensor colorSensor;


    @Override
    public void init() {

        encoder = hardwareMap.get(AnalogInput.class, "indexerPOS");
//        commandScheduler = CommandScheduler.getInstance();
//        commandScheduler.init(this);
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

        telemetry.addData("pos", encoder.getVoltage() * (360 / encoder.getMaxVoltage()));

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
//        commandScheduler.run();
//        commandScheduler.updateTelemetry();


    }
}
