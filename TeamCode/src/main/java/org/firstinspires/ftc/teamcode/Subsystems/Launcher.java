package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;
import org.firstinspires.ftc.teamcode.Controllers.FeedforwardController;
import org.firstinspires.ftc.teamcode.Controllers.PIDController;
import org.firstinspires.ftc.teamcode.Config.PIDCoefficients;

public class Launcher implements Subsystem {
    //    private PIDController pidL;
//    private PIDController pidR;
    private PIDController pid;
    private FeedforwardController feedforward;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Telemetry telemetry;
    private double targetRPM = 0;
    private double leftrpm = 0;
    private double rightrpm = 0;
    private double currentLeftRPM = 0;
    private double currentRightRPM = 0;
    private long motorWar = 0;


    private final int TICKS_PER_REVOLUTION = 28;
    private boolean isSpinningFlag = false;

    public Launcher(HardwareMap hMap, Telemetry telemetry) {
        pid = new PIDController(PIDCoefficients.LP, PIDCoefficients.LI, PIDCoefficients.LD, false);
//        pidL = new PIDController(PIDCoefficients.LLP,PIDCoefficients.LLI,PIDCoefficients.LLD, false);
//        pidR = new PIDController(PIDCoefficients.LRP,PIDCoefficients.LRI,PIDCoefficients.LRD, false);
        feedforward = new FeedforwardController(PIDCoefficients.LKS, PIDCoefficients.LKV);
//        pid.setInverted(true);
        leftMotor = hMap.get(DcMotorEx.class, "launcherLeft");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor = hMap.get(DcMotorEx.class, "launcherRight");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    private void setTargetRPM(double rpm){
        pid.setTarget(rpm);
//        pidL.setTarget(rpm);
//        pidR.setTarget(rpm);
        targetRPM = rpm;
    }
    private void update(){
        leftrpm = getCurrentRPM(LauncherWheel.LEFT);
        rightrpm = getCurrentRPM(LauncherWheel.RIGHT);
//        leftMotor.setPower(pid.update(leftrpm) + feedforward.calculateWithVelocities(targetRPM));
//        double rightrpm = getCurrentRPM(LauncherWheel.RIGHT);
//        rightMotor.setPower(pid.update(rightrpm) + feedforward.calculateWithVelocities(targetRPM));
        double fFtarget = feedforward.calculateWithVelocities(targetRPM);
        currentLeftRPM = pid.update(leftrpm);
        //currentRightRPM = pid.update(rightrpm);
        leftMotor.setPower(currentLeftRPM + fFtarget);
        rightMotor.setPower(currentLeftRPM + fFtarget);
        isSpinningFlag = true;
    }
    public enum LauncherWheel {
        LEFT,
        RIGHT
    }
    public double getCurrentRPM(LauncherWheel wheel){
        switch(wheel){
            case LEFT:
                return 60*leftMotor.getVelocity()/TICKS_PER_REVOLUTION;
            case RIGHT:
                return 60*rightMotor.getVelocity()/TICKS_PER_REVOLUTION;
        }
        return 0;
    }
    public boolean isSpinning(){
        return isSpinningFlag;
    }
    public void stopPid() {
        isSpinningFlag = false;
        pid.stop();
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        targetRPM = 0;
    }
    public Command runLauncher(){
//        return new InstantCommand(()-> setPower(1));
        return new FunctionalCommand(
                ()->setTargetRPM(targetRPM),
                this::update,
                (interrupted)->stopPid(),
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
    /*public Command stop(){
        return new InstantCommand(this::stopPid,this);
    }*/

    public void updateTelemetry() {
        telemetry.addData("Left wheel rpm", getCurrentRPM(LauncherWheel.LEFT));
        telemetry.addData("Right wheel rpm", getCurrentRPM(LauncherWheel.RIGHT));
        telemetry.addData("Left wheel current", leftMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Right wheel current", rightMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("target PID:", pid.getTarget());
        telemetry.addData("leftRPM:", leftrpm);
        telemetry.addData("rightRPM:", rightrpm);
        telemetry.addData("targetRPM:", targetRPM);
        telemetry.addData("isSpinningFlag",isSpinningFlag);

    }

    public boolean areMotorsFighting() {
        if (leftMotor.getCurrent(CurrentUnit.AMPS) > 9 && rightMotor.getCurrent(CurrentUnit.AMPS) > 9 && motorWar == -1) {
            motorWar = System.currentTimeMillis();
        } else if (leftMotor.getCurrent(CurrentUnit.AMPS) <= 9 && rightMotor.getCurrent(CurrentUnit.AMPS) <= 9 && motorWar != -1) {
            motorWar = -1;
        }
        return System.currentTimeMillis() - motorWar > 5000 && motorWar != -1;
    }

}
