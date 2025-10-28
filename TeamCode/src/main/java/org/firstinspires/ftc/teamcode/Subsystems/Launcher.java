package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;
import org.firstinspires.ftc.teamcode.PIDController;

public class Launcher implements Subsystem {
    private PIDController pidL;
    private PIDController pidR;
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Telemetry telemetry;
    private double targetRPM = 0;


    private final int TICKS_PER_REVOLUTION = 28;
    private boolean isSpinningFlag = false;

    public Launcher(HardwareMap hMap, Telemetry telemetry) {
        // Left and right from the servo side, not ramp side
        pidL = new PIDController(0.008,0,0);
        pidR = new PIDController(0.008,0,0);
        leftMotor = hMap.get(DcMotorEx.class, "flywheelLeft");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hMap.get(DcMotorEx.class, "flywheelRight");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        pidL.setTarget(rpm);
        pidR.setTarget(rpm);
        targetRPM = rpm;
    }
    private void update(){
        double leftrpm = getCurrentRPM(LauncherWheel.LEFT);
        leftMotor.setPower(pidL.update(leftrpm));
        double rightrpm = getCurrentRPM(LauncherWheel.RIGHT);
        rightMotor.setPower(pidR.update(rightrpm));
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
        return new InstantCommand(this::stopPid,this);
    }

    public void updateTelemetry() {
        telemetry.addData("Left wheel rpm", getCurrentRPM(LauncherWheel.LEFT));
        telemetry.addData("Right wheel rpm", getCurrentRPM(LauncherWheel.RIGHT));
    }

}
