package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class Trigger {

    Command[] pressed = new Command[] {};

    Command[] justPressed = new Command[] {};

    Command[] justReleased = new Command[] {};

    Command[] released = new Command[] {};

    public Command[] getOnPressed(){
        return pressed;
    }
    public Command[] getOnJustPressed(){
        return justPressed;
    }
    public Command[] getOnJustReleased(){
        return justReleased;
    }
    public Command[] getOnReleased(){
        return released;
    }
    public Trigger onPressed(Command... commands){
        pressed = commands;
        return this;
    }
    public Trigger onJustPressed(Command... commands){
        justPressed = commands;
        return this;
    }
    public Trigger onJustReleased(Command... commands){
        justReleased = commands;
        return this;
    }
    public Trigger  onReleased(Command... commands){
        released = commands;
        return this;
    }

    //commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY)

    // Methods:
    // .onJustPressed(...), onRelease(...);



}
