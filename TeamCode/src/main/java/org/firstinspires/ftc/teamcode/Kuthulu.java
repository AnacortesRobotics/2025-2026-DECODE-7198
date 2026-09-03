package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis2Wheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake1Motor;

@TeleOp
public class Kuthulu extends OpMode {
//    DcMotor leftdrive;
//    DcMotor rightdrive;
    CommandScheduler commandScheduler;
    Chassis2Wheel chassis2Wheel;
    Intake1Motor intake1Motor;
    @Override
    public void init(){
//        leftdrive = hardwareMap.get(DcMotor.class, "leftdrive");
//        leftdrive.setDirection(DcMotor.Direction.REVERSE);
//        rightdrive = hardwareMap.get(DcMotor.class, "rightdrive");
//        rightdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);
        chassis2Wheel = new Chassis2Wheel(hardwareMap, telemetry);
        intake1Motor = new Intake1Motor(hardwareMap, telemetry);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.update();

        commandScheduler.setDefaultCommands(new InstantCommand(()-> chassis2Wheel.chassis2WheelDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x)));
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.A_BUTTON, CommandScheduler.GamepadIndex.PRIMARY).onJustPressed(intake1Motor.intakeIn());
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.B_BUTTON, CommandScheduler.GamepadIndex.PRIMARY).onJustPressed(intake1Motor.intakeOut());
        commandScheduler.getTrigger(CommandScheduler.GamepadInput.X_BUTTON, CommandScheduler.GamepadIndex.PRIMARY).onJustPressed(intake1Motor.intakeStop());
    }





    @Override
    public void loop() {
        commandScheduler.run();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }
}
