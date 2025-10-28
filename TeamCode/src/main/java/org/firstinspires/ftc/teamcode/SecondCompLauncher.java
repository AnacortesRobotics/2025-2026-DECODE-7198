package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Thread.sleep;


public class SecondCompLauncher implements Subsystem {
    private PIDController pidL;
    private PIDController pidR;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Telemetry telemetry;
    private CRServo indexer;
    public Servo loader;
    private double targetRPM = 0;


    private final int TICKS_PER_REVOLUTION = 28;
    private boolean isSpinningFlag = false;

    public SecondCompLauncher(HardwareMap hMap, Telemetry telemetry) {// the error is fine if no error delete this
        // Left and right from the servo side, not ramp side
        pidL = new PIDController(0.002,0,0);
        pidR = new PIDController(0.002,0,0);
//        leftMotor = hMap.get(DcMotorEx.class, "flywheelLeft");
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor = hMap.get(DcMotorEx.class, "flywheelRight");
//        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        indexer = hMap.get(CRServo.class, "indexerServo");
        loader = hMap.get(Servo.class, "loaderServo");
        this.telemetry = telemetry;
    }
    public void setPower(double power){
        rightMotor.setPower(power);
        leftMotor.setPower(power);
    }
    public Command chargeLauncher(double power) {
        return new InstantCommand(
                () -> setPower(power)
        ).setInterruptable(true);
    }
    public Command loadToLauncher(){
//        loader.setPosition(.25);
        this.telemetry.addData("we got to here",0.25);

        return new SequentialCommandGroup(
                new InstantCommand(
                () -> loader.setPosition(0.25)
        ),
        new WaitCommand(
                250
        ),
        new InstantCommand(
                ()->loader.setPosition(1)
        ),
        new WaitCommand(
                1000
        ),
         new InstantCommand(
                ()->loader.setPosition(0)
        ).setInterruptable(true)
        );

//        return new InstantCommand(
//                () -> loader.setPosition(0.25)
//        ).setInterruptable(true);




    }
    private void setTargetRPM(double rpm){
//        pidL.setTarget(rpm);
//        pidR.setTarget(rpm);
//        targetRPM = rpm;
    }
    private void update(){
//        double leftrpm = getCurrentRPM(Launcher.LauncherWheel.LEFT);
//        leftMotor.setPower(pidL.update(leftrpm));
//        double rightrpm = getCurrentRPM(Launcher.LauncherWheel.RIGHT);
//        rightMotor.setPower(pidR.update(rightrpm));
//        isSpinningFlag = true;
    }
    public enum LauncherWheel {
        LEFT,
        RIGHT
    }
    public double getCurrentRPM(Launcher.LauncherWheel wheel){
//        switch(wheel){
//            case LEFT:
//                return 60*leftMotor.getVelocity()/TICKS_PER_REVOLUTION;
//            case RIGHT:
//                return 60*rightMotor.getVelocity()/TICKS_PER_REVOLUTION;
//        }
        return 0;
    }
    public boolean isSpinning(){
        return isSpinningFlag;
    }
    public void stopPid() {
        isSpinningFlag = false;
        pidL.stop();
        pidR.stop();
        setPower(0);
    }
    public Command start(){
        return new FunctionalCommand(
                ()->setTargetRPM(targetRPM),
                this::update,
                (interrupted)->{},
                ()->false,
                this).setInterruptable(true);
    }
    public Command setRPM(double rpm){
        return new InstantCommand(()->setTargetRPM(rpm));
    }
    public Command adjustRPM(double increment){
        return new InstantCommand(
                ()->setTargetRPM(targetRPM + increment));
    }
    public Command stop(){
        return new InstantCommand(()->stopPid(),this);
    }
}
