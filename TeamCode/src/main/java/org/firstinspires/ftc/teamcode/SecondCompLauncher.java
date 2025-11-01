package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.Console;

import static java.lang.Thread.sleep;


public class SecondCompLauncher implements Subsystem {
    private PIDController pidL;
    private PIDController pidR;
    private PIDController indexerPid;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Telemetry telemetry;
    private CRServo indexer;
    private AnalogInput indexerPosition;
    public Servo loader;
    public double pastPos;
    public double nowPos;
    private Direction direction;
    private double targetRPM = 0;


    private final int TICKS_PER_REVOLUTION = 28;
    private boolean isSpinningFlag = false;

    public SecondCompLauncher(HardwareMap hMap, Telemetry telemetry) {
        // Left and right from the servo side, not ramp side
        pidL = new PIDController(0.002,0,0, false);
        pidR = new PIDController(0.002,0,0, false);
        indexerPid = new PIDController(.01, 0, 0, false);
//        leftMotor = hMap.get(DcMotorEx.class, "flywheelLeft");
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor = hMap.get(DcMotorEx.class, "flywheelRight");
//        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexer = hMap.get(CRServo.class, "indexerServo");
        indexerPosition = hMap.get(AnalogInput.class, "indexerPOS");
        loader = hMap.get(Servo.class, "loaderServo");
        this.direction = Direction.FORWARD;
        this.telemetry = telemetry;
    }
//    public boolean spindexer(){
//        //1 create pid controller
//        //2 get position using second port
//        //3
//
//    }
    public Command turnSpindexer(){
        indexerPid.setTarget(indexerPid.getTarget());
//        indexer.setPower(indexerPid.update(indexerPosition));
        telemetry.addData("position: ", indexerPosition);
        return new InstantCommand(
                () -> indexer.setPower(0.05)
        );

    }
    public Command stopTurningSpindexer(){
        if (nowPos == 0) {
            return new InstantCommand(
                    () -> indexer.setPower(0)
            );
        }
        else {
            return new InstantCommand(
                    ()-> indexer.setPower(0.05)
            );
}
    }
    public enum Direction {
        FORWARD,
        REVERSE
    }
    public double getServoPosition(){
        if (indexerPosition == null) {
            pastPos = 0;
        }
        else {
            pastPos = nowPos;
            telemetry.addData("past position",pastPos);
        }
        nowPos = (indexerPosition.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
        return nowPos;
        //        return (indexerPosition.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
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
        telemetry.addData("in y button past position", pastPos);
        telemetry.addData("in y button now position", nowPos);
    }
    public enum LauncherWheel {
        LEFT,
        RIGHT
    }
    public double getCurrentRPM(SecondCompLauncher.LauncherWheel wheel){
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
