package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.*;

public class RgbLed implements Subsystem {

    private Servo led;

    private Telemetry telemetry;

    public enum Color {
        OFF,
        RED,
        ORANGE,
        YELLOW,
        SAGE,
        GREEN,
        AZURE,
        BLUE,
        INDIGO,
        VIOLET,
        WHITE
    }

    public void init(HardwareMap hMap, Telemetry telemetry) {
        led = hMap.get(Servo.class, "led");

        this.telemetry = telemetry;
    }

    public void setColor(Color color) {
        switch (color) {
            case OFF:
                led.setPosition(0);
                break;
            case RED:
                led.setPosition(0.28);
                break;
            case ORANGE:
                led.setPosition(0.333);
                break;
            case YELLOW:
                led.setPosition(0.388);
                break;
            case SAGE:
                led.setPosition(0.444);
                break;
            case GREEN:
                led.setPosition(0.5);
                break;
            case AZURE:
                led.setPosition(0.555);
                break;
            case BLUE:
                led.setPosition(0.611);
                break;
            case INDIGO:
                led.setPosition(0.666);
                break;
            case VIOLET:
                led.setPosition(0.722);
                break;
            case WHITE:
                led.setPosition(1);
                break;
        }
    }

    public Command blinkOnce(long ms, Color color) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new InstantCommand(()->setColor(color), this),
                new WaitCommand(ms),
                new InstantCommand(()->setColor(Color.OFF), this),
                new WaitCommand(ms));

        return commandGroup;
    }

    public Command blink(long ms, Color color) {
        return new RepeatCommand(blinkOnce(ms, color), 3);
    }

    public Command blink(long ms, Color color, String name) {
        RepeatCommand command = new RepeatCommand(blinkOnce(ms, color), 3);
        command.setName(name);
        return command;
    }

}
