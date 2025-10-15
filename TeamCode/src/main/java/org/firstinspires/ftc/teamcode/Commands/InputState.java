package org.firstinspires.ftc.teamcode.Commands;

public class InputState {
    public enum InputType{
        DIGITAL,
        ANALOG
    }
    private double value;
    private double lastValue;
    private InputType inputType;
    private double threshold;
    public InputState(double value,double lastValue, double threshold) {
        this.value = value;
        this.inputType = InputType.ANALOG;
        this.lastValue = lastValue;
        this.threshold = threshold;
    }
    public InputState(double value,double lastValue){
        this(value, lastValue, 0);
    }
    public InputState(boolean value, boolean lastValue){
        this.value = (value ? 1:0);
        this.lastValue = (lastValue ? 1:0);
        this.inputType = (InputType.DIGITAL);
        this.threshold = 0;
    }
    public boolean isPressed(){
        return value>=threshold;
    }
    public boolean justPressed(){
        return value>=threshold && threshold > lastValue;
    }
    public boolean isReleased(){
        return value<threshold;
    }
    public boolean justReleased(){
        return value<threshold && threshold <= lastValue;
    }
    public double getValue(){
        return value;
    }
}
