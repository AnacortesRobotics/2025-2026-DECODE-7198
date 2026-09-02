package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;

@TeleOp
public class TestCRServoTeleop extends OpMode {
    // 1. Declare subsystem variables
    private CommandScheduler commandScheduler;
    private TestCRServo testCRServo;

    // 2. Declare local variables needed

    @Override
    public void init() {

        // 1. Declare subsystem classes to be used in TeleOp
        testCRServo = new TestCRServo(hardwareMap, telemetry);

        // 2. Declare a singleton instance of CommandScheduler
        commandScheduler = CommandScheduler.getInstance();

        // 3. Init the CommandScheduler
        commandScheduler.init(this);

        // 4. Tell robot what commands to run
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.Y_BUTTON, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(testCRServo.stopTurningSpindexer());
        commandScheduler
                .getTrigger(CommandScheduler.GamepadInput.X_BUTTON, CommandScheduler.GamepadIndex.PRIMARY)
                .onJustPressed(testCRServo.turnSpindexer());
    }

    @Override
    public void loop() {
        commandScheduler.run();
        commandScheduler.updateTelemetry();
        testCRServo.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }
}
