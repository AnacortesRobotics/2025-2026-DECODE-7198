package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

@Autonomous
public class CommandTest extends OpMode {

    private RgbLed ledController;
    private Command blinkSage;
    private Command blinkRed;

    private CommandScheduler commandScheduler;

    private Launcher launcher;

    private double launcherPower = 0;

    @Override
    public void init() {
        ledController = new RgbLed();
        ledController.init(hardwareMap, telemetry);

        launcher.init(hardwareMap, telemetry);

//        blinkSage = ledController.blink(500, RgbLed.Color.SAGE, "Blink sage");
//
//        blinkRed = ledController.blink(500, RgbLed.Color.RED, "Blink red");
//
//        commandScheduler = CommandScheduler.getInstance();
//
//        commandScheduler.init(telemetry);
//
//        commandScheduler.schedule(blinkSage, blinkRed);
    }

    @Override
    public void loop() {
//        if (gamepad1.right_trigger > .5) {
//            commandScheduler.schedule(launcher.loadBall());
//        } else if (gamepad1.dpad_up) {
//            launcherPower += .25;
//            commandScheduler.schedule(launcher.chargeLauncher(launcherPower));
//        } else if (gamepad1.dpad_down) {
//            launcherPower -= .25;
//            commandScheduler.schedule(launcher.chargeLauncher(launcherPower));
//        } else if (gamepad1.b) {
//            commandScheduler.schedule(launcher.chargeLauncher(0));
//        }




        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }

}
