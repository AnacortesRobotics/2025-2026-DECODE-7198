package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;

public class Launcher implements Subsystem {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private CRServo leftServo;
    private CRServo rightServo;

    private Telemetry telemetry;

    private Loader loader;
    private Shooter shooter;

    public void init(HardwareMap hMap, Telemetry telemetry) {
        // Left and right from the servo side, not ramp side
        leftMotor = hMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor = hMap.get(DcMotor.class, "rightMotor");

        leftServo = hMap.get(CRServo.class, "leftServo");
        rightServo = hMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);

        this.telemetry = telemetry;

        loader = new Loader();
        shooter = new Shooter();
    }

    public Command loadBall() {

        return new SequentialCommandGroup(
                new InstantCommand(()->loader.setServoPower(1), loader),
                new WaitCommand(1000),
                new InstantCommand(()->loader.setServoPower(0), loader)
        ).setInterruptable(true);
    }

    public Command chargeLauncher(double power) {
        return new InstantCommand(
                ()->shooter.setMotorPower(power),
                shooter
        ).setInterruptable(true);
    }

    public class Loader implements Subsystem {

        private void setServoPower(double power) {
            leftServo.setPower(power);
            rightServo.setPower(power);
        }
    }

    public class Shooter implements Subsystem {

        private void setMotorPower(double power) {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

}
