
package org.firstinspires.ftc.teamcode;

import android.widget.Button;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
// import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis2Wheel;

import static com.sun.tools.doclint.Entity.or;
// import org.firstinspires.ftc.teamcode.Subsystems.LimelightArtifact;
// import org.firstinspires.inspection.GamepadInspection;

@TeleOp(name = "MiniBot")

public class MiniBot extends OpMode {
    DcMotorEx leftdrive;
    DcMotorEx rightdrive;

    DcMotorEx arm;
   // Telemetry telemetry;
    //MultipleTelemetry telemetry;

    private Servo grabServo;
    private Button armSwitch;
    double grabPos;

    boolean last = false;
    boolean autoTurnState = false;
    double turn;

    Chassis2Wheel chassis2Wheel;
    CommandScheduler commandScheduler;
    


    private Command armUp(){
        return new  InstantCommand(()->arm.setPower(.65));


    }
    private Command armDown(){
        return new SequentialCommandGroup(new InstantCommand(()->arm.setPower(-.2)), new WaitCommand(150), new InstantCommand(()->arm.setPower(0)));
    }
    private Command armStop(){
        return new InstantCommand(()-> arm.setPower(0));
    }

    private Command waitUntil(){
        return new FunctionalCommand(()->{}, ()->{}, (interrupted)->{}, ()->{return arm.getCurrentPosition() > 134;});
    }

    private Command graberIn(){
        return new InstantCommand(()->grabServo.setPosition(1));
    }

    private Command graberOut(){
        return new InstantCommand(()->grabServo.setPosition(.5));
    }

    private Command pickUpStuff(){
        return new SequentialCommandGroup(
                graberIn(),
                new WaitCommand(300),
                armUp(),
                waitUntil(),
                new WaitCommand(200),
                armStop(),
                graberOut()
        );
    }

    public void init(){
        leftdrive = hardwareMap.get(DcMotorEx.class, "leftdrive");
        leftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightdrive = hardwareMap.get(DcMotorEx.class, "rightdrive");
        rightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabServo = hardwareMap.get(Servo.class, "graber");
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        commandScheduler = CommandScheduler.getInstance();
        telemetry.setMsTransmissionInterval(11);
        telemetry.update();
        chassis2Wheel = new Chassis2Wheel(hardwareMap, telemetry);

        commandScheduler.setDefaultCommands(new InstantCommand(()->chassis2Wheel.move(-gamepad1.left_stick_y, gamepad1.right_stick_x)));
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.B_BUTTON, CommandScheduler.GamepadIndex.PRIMARY).onPressed(armDown());
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.X_BUTTON, CommandScheduler.GamepadIndex.PRIMARY).onPressed(armStop());
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.RIGHT_BUMPER, CommandScheduler.GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->grabServo.setPosition(.5)));
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.LEFT_BUMPER, CommandScheduler.GamepadIndex.PRIMARY).onJustPressed(pickUpStuff());
    }

    @Override
    public void loop(){

//        chassis2Wheel.move(-gamepad1.left_stick_y, gamepad1.right_stick_x);



        telemetry.addData("arm Power", arm.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("ticks", arm.getCurrentPosition());
        telemetry.addData("right power",rightdrive.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left power",leftdrive.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("left stick y",gamepad1.left_stick_y);
        telemetry.update();
        commandScheduler.run();
        }
    }
