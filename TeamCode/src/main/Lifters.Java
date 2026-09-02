


public class Lifters implements Subsystem{
    private Servo leftServo;
    private Servo rightServo;

    public Lifters(HardwareMap hMap, Telemetry telemetry){
        leftServo = hMap.get(Servo.class, "leftServo");
        rightServo = hMap.get(Servo.class, "rightServo");
    }

    public Command moveLifters(double position){
        return new ParallelComandGroup( new InstantCommand(()->leftServo.setPosition(position)),new InstantCommand(()->rightServo.setPosition(position)));
    }


}