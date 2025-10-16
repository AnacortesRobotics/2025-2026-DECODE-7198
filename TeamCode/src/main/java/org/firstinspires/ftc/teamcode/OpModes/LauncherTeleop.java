package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

@TeleOp
public class LauncherTeleop extends OpMode {
    private Chassis chassis;
    private CommandScheduler commandScheduler;
    private Launcher launcher;
    private final double RPM_INCREMENTS = 50;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap, telemetry);

        commandScheduler = CommandScheduler.getInstance();

        commandScheduler.init(this);
        chassis = new Chassis(hardwareMap, telemetry, false);

        commandScheduler
                .getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY)
                .onJustPressed(launcher.start());
        commandScheduler
                .getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY)
                .onJustPressed(launcher.stop());
        commandScheduler
                .getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY)
                .onJustPressed(launcher.adjustRPM(RPM_INCREMENTS));
        commandScheduler
                .getTrigger(GamepadInput.LEFT_BUMPER, GamepadIndex.PRIMARY)
                .onJustPressed(launcher.adjustRPM(-RPM_INCREMENTS));
        commandScheduler.setDefaultCommands(
                new InstantCommand(()->
                        chassis.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), chassis));
    }

    @Override
    public void loop() {
//      telemetry.addData("target rpm", targetLauncherRpm);
        telemetry.addData("left rpm", launcher.getCurrentRPM(Launcher.LauncherWheel.LEFT));
        telemetry.addData("right rpm", launcher.getCurrentRPM(Launcher.LauncherWheel.RIGHT));
        telemetry.addData("Is it working ", launcher.isSpinning());

        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }

}
