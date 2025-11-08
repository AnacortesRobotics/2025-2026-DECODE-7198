package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;

public class Intake implements Subsystem {

    private CRServo intakeMotor;
    private final double POWER = .5;

    private Telemetry telemetry;


    public Intake(HardwareMap hMap, Telemetry telemetry) {

        intakeMotor = hMap.get(CRServo.class, "intakeMotor");

        this.telemetry = telemetry;

    }

    public Command turnIntake() {// run when pressing bumper
        return new InstantCommand (
                () -> setIntakePower(POWER)
        );
    }

    public Command stopIntake() {// run when not pressing bumper
        return new InstantCommand (
                () -> setIntakePower(0)
        );
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
}
