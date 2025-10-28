package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;

@TeleOp
public class SecondCompLauncherTeleop extends OpMode {
    private Chassis chassis;
    private CommandScheduler commandScheduler;
    private SecondCompLauncher secondCompLauncher;
    private final double RPM_INCREMENTS = 50;
    @Override
    public void init() {

        commandScheduler = CommandScheduler.getInstance();

        commandScheduler.init(this);
        //chassis = new Chassis(hardwareMap, telemetry, false);

        secondCompLauncher = new SecondCompLauncher(hardwareMap, telemetry);

        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.A_BUTTON, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(secondCompLauncher.start());
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.B_BUTTON, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(secondCompLauncher.stop());
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.Y_BUTTON, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(secondCompLauncher.loadToLauncher());
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.RIGHT_BUMPER, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(secondCompLauncher.adjustRPM(RPM_INCREMENTS));
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.LEFT_BUMPER, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(secondCompLauncher.adjustRPM(-RPM_INCREMENTS));
        commandScheduler.setDefaultCommands(
                new InstantCommand(()->
                        chassis.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), chassis));
    }
    @Override
    public void loop() {
//      telemetry.addData("target rpm", targetLauncherRpm);
//        telemetry.addData("left rpm", SecondCompLauncher.getCurrentRPM(Launcher.LauncherWheel.LEFT));
//        telemetry.addData("right rpm", SecondCompLauncher.getCurrentRPM(Launcher.LauncherWheel.RIGHT));
//        telemetry.addData("Is it working ", SecondCompLauncher.isSpinning());
        telemetry.addData("servo", secondCompLauncher.loader.getPosition());

        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

}
