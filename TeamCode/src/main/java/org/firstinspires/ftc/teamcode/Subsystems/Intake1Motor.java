package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;

public class Intake1Motor implements Subsystem {

    DcMotor intake;

    public Intake1Motor(HardwareMap hMap, Telemetry telemetry){
        intake = hMap.get(DcMotor.class, "intakeMotor");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Command intakeIn(){return new InstantCommand(()->intake.setPower(.4));}

    public Command intakeOut(){return new InstantCommand(()->intake.setPower(-.4));}

    public Command intakeStop(){
        return new InstantCommand(()->intake.setPower(0));
    }
}
