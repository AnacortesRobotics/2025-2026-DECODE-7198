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
import org.firstinspires.ftc.teamcode.ValueTurnover;

@TeleOp
public class DecodeTeleOp extends OpMode {

    Chassis chassis;
    LinearTrajectory trajectory;
    Launcher launcher;
    Indexer indexer;
    CommandScheduler commandScheduler;
    ValueTurnover valueTurnover;

    double forward = 0;
    double strafe = 0;
    double rotate = 0;

    boolean isRed;


    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        valueTurnover = ValueTurnover.getInstance();
        commandScheduler.init(this);
        isRed = valueTurnover.getIsRed();

        //Command driveToStart = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        Command targetAngle = chassis.autoTurn(()->forward, ()->strafe, isRed ? new Pose2D(DistanceUnit.INCH, 71, -71, AngleUnit.DEGREES, 0) :
                new Pose2D(DistanceUnit.INCH, 71, 71, AngleUnit.DEGREES, 0));
//        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY).
//                onJustPressed(driveToStart);
//        commandScheduler.getTrigger(GamepadInput.LEFT_BUMPER, GamepadIndex.PRIMARY).
//                onJustPressed(driveToStart.cancel());
        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(targetAngle);
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(targetAngle.cancel());
        commandScheduler.getTrigger(GamepadInput.RIGHT_TRIGGER, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.setMaxSpeed(.4)));
        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.setMaxSpeed(1)));
        //commandScheduler.getTrigger(GamepadInput.RIGHT_TRIGGER, GamepadIndex.PRIMARY).onJustReleased(new InstantCommand(()->chassis.setMaxSpeed(1)));
        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                launcher.setRPM(4640), launcher.start()
        ).setInterruptable(true));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                launcher.setRPM(4960), launcher.start()
        ).setInterruptable(true));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.SECONDARY).onJustPressed(launcher.stop().setName("Stop launcher"));
        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.SECONDARY).onJustPressed(indexer.fireBall());
        commandScheduler.setDefaultCommands(new InstantCommand(
                ()->chassis.mecanumDriveFieldCentric(forward, strafe, rotate),
                chassis));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(valueTurnover.getCurrentPos());
        chassis.setHolonomicOffset(isRed ? -Math.PI / 2 : Math.PI / 2);
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
