package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;

public class Chassis2Wheel implements Subsystem {

    public DcMotor leftDrive;
    public DcMotor rightDrive;



    public Chassis2Wheel(HardwareMap hMap, Telemetry telemetry){
        leftDrive = hMap.get(DcMotor.class, "leftdrive");
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive = hMap.get(DcMotor.class, "rightdrive");
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void drive (double forward){
        if (forward>.4){
            forward=0.4;
        }
        if (forward<-.4){
            forward=-0.4;
        }
        leftDrive.setPower(forward);
        rightDrive.setPower(forward);
    }

    private void turn (double turning){
        if (turning>.3){
            turning=0.3;
        }
        if (turning<-.3){
            turning=-0.3;
        }
        leftDrive.setPower(turning);
        rightDrive.setPower(-turning);
    }
    private void stopDrive (){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void chassis2WheelDrive(double forward, double rotate){
        if (forward>.4){
            forward=0.4;
        }
        else if (forward<-.4){
            forward=-0.4;
        }
        if (rotate>.3){
            rotate=0.3;
        }
        else if (rotate<-.3){
            rotate=-0.3;
        }
        leftDrive.setPower(forward+rotate);
        rightDrive.setPower(forward-rotate);
    }
    public void move(double forwards, double turns){
        if (forwards > 0.1 || forwards < -0.1 ) {
            drive(forwards);
        }
        else stopDrive();
        if(turns > 0.1 || turns < -0.1){
            turn(turns);
        }
        else stopDrive();
    }
}
