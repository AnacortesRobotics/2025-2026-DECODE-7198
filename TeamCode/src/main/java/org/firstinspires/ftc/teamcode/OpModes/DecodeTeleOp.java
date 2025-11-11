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
import org.firstinspires.ftc.teamcode.Controllers.LinearTrajectory;
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
        Command driveToLoad = chassis.driveToPosition(isRed ? new Pose2D(DistanceUnit.INCH, 63, -63, AngleUnit.DEGREES, -45)
                : new Pose2D(DistanceUnit.INCH, -63, -63, AngleUnit.DEGREES, 45)).setInterruptable(true);
        Command driveToShootFar = chassis.driveToPosition(isRed ?
                new Pose2D(DistanceUnit.INCH, -69 + chassis.ROBOT_LENGTH / 2.0, -25 + chassis.ROBOT_WIDTH / 2.0, AngleUnit.DEGREES, -20) :
                new Pose2D(DistanceUnit.INCH, -69 + chassis.ROBOT_LENGTH / 2.0, 25 - chassis.ROBOT_WIDTH / 2.0, AngleUnit.DEGREES, 21)).setInterruptable(true);
        Command driveToShootMiddle = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, isRed ? -55 : 55)).setInterruptable(true);
        Command driveToPark = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -39, isRed ? 33 : -33, AngleUnit.DEGREES, 0)).setInterruptable(true);
        commandScheduler.getTrigger(GamepadInput.DPAD_DOWN, GamepadIndex.PRIMARY).onJustPressed(driveToLoad);
        commandScheduler.getTrigger(GamepadInput.DPAD_UP, GamepadIndex.PRIMARY).onJustPressed(driveToShootFar);
        commandScheduler.getTrigger(GamepadInput.DPAD_LEFT, GamepadIndex.PRIMARY).onJustPressed(driveToShootMiddle);
        commandScheduler.getTrigger(GamepadInput.DPAD_RIGHT, GamepadIndex.PRIMARY).onJustPressed(driveToPark);
        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new SequentialCommandGroup(
                driveToLoad.cancel(), driveToShootFar.cancel(), driveToShootMiddle.cancel(), driveToPark.cancel()));

//        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
//                launcher.setRPM(4540), launcher.start()
//        ).setInterruptable(true));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                launcher.setRPM(4910), launcher.start()
        ).setInterruptable(true));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.SECONDARY).onJustPressed(launcher.stop().setName("Stop launcher"));
        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.SECONDARY).onJustPressed(indexer.fireBall());

        commandScheduler.getTrigger(GamepadInput.START_BUTTON, GamepadIndex.PRIMARY).onJustPressed(
                new InstantCommand(()->commandScheduler.setDefaultCommands(new InstantCommand(
                        ()->chassis.mecanumDriveFieldCentric(forward, strafe, rotate),
                        chassis)))
        );
        commandScheduler.getTrigger(GamepadInput.BACK_BUTTON, GamepadIndex.PRIMARY).onJustPressed(
                new InstantCommand(()->commandScheduler.setDefaultCommands(new InstantCommand(
                        ()->chassis.mecanumDrive(forward, strafe, rotate),
                        chassis)))
        );

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
