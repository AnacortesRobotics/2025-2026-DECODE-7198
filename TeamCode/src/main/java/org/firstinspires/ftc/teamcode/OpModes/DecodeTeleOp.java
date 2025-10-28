package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.LinearTrajectory;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

@TeleOp
public class DecodeTeleOp extends OpMode {

    Chassis chassis;
    LinearTrajectory trajectory;
    Launcher launcher;
    Indexer indexer;
    CommandScheduler commandScheduler;

    double forward = 0;
    double strafe = 0;
    double rotate = 0;


    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);

        Command driveToStart = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        Command targetAngle = chassis.autoTurn(()->forward, ()->strafe, new Pose2D(DistanceUnit.INCH, 70, -70, AngleUnit.DEGREES, 0));
        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY).
                onJustPressed(driveToStart);
        commandScheduler.getTrigger(GamepadInput.LEFT_BUMPER, GamepadIndex.PRIMARY).
                onJustPressed(driveToStart.cancel());
        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(targetAngle);
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(targetAngle.cancel());
        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new SequentialCommandGroup(
                launcher.chargeLauncher(.78)
        ));
        commandScheduler.getTrigger(GamepadInput.DPAD_UP, GamepadIndex.PRIMARY).onJustPressed(new SequentialCommandGroup(
                launcher.chargeLauncher(.82)
        ));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.PRIMARY).onJustPressed(launcher.stop().setName("Stop launcher"));
        commandScheduler.getTrigger(GamepadInput.RIGHT_TRIGGER, GamepadIndex.PRIMARY).onJustPressed(indexer.fireBall());
        commandScheduler.setDefaultCommands(new InstantCommand(
                ()->chassis.mecanumDriveFieldCentric(forward, strafe, rotate),
                chassis));
    }

    @Override
    public void loop() {
        chassis.updateOdo();

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        commandScheduler.run();
        commandScheduler.updateTelemetry();
        chassis.updateTelemetry();
        launcher.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }
}
