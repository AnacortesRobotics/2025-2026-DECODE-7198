package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.LinearTrajectory;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.ValueTurnover;

@Autonomous
public class DecodeAutoRedFar extends OpMode {

    Chassis chassis;
    LinearTrajectory trajectory;
    Launcher launcher;
    Indexer indexer;
    CommandScheduler commandScheduler;
    ValueTurnover valueTurnover;

    // 0,0 is the center

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        valueTurnover = ValueTurnover.getInstance();
        commandScheduler.init(this);

        Command turnToShoot = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -69 + chassis.ROBOT_LENGTH / 2.0, -25 + chassis.ROBOT_WIDTH / 2.0, AngleUnit.DEGREES, -20)).setName("Turn To Position");
        Command prepareLauncher = launcher.setRPM(4950).setName("Prepare Launcher");
        Command launchBalls = new SequentialCommandGroup(
                //new WaitCommand(800),
                new RepeatCommand(
                        new SequentialCommandGroup(indexer.fireBall(), new WaitCommand(1500)), 4)).
                addRequirements(chassis).setName("Launch Balls").setInterruptable(false);
        Command moveToEnd = chassis.driveToPosition(
                new Pose2D(DistanceUnit.INCH, -72 + chassis.ROBOT_LENGTH / 2.0, -48 + chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0)
        ).setName("Move to End");

        commandScheduler.schedule(turnToShoot, launcher.start(), prepareLauncher, launchBalls,
                new InstantCommand(()->chassis.setMaxSpeed(.4)).setName("Slow down chassis").addRequirements(chassis),
                launcher.stop().addRequirements(chassis),
                moveToEnd,
                new InstantCommand(()-> chassis.stop(), chassis));
                //new InstantCommand(this::terminateOpModeNow).addRequirements(chassis));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, -72 + chassis.ROBOT_LENGTH / 2.0, -24 + chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0));
        //chassis.setMaxSpeed(.9);
        //launcher.setRPM(4800);
        //commandScheduler.schedule(new RepeatCommand(new SequentialCommandGroup(chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)).setName("DriveFar"),
        //        chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)).setName("DriveBack"))));

    }

    @Override
    public void loop() {
        chassis.updateOdo();

        commandScheduler.run();

        commandScheduler.updateTelemetry();
        chassis.updateTelemetry();
        launcher.updateTelemetry();
    }

    @Override
    public void stop() {
        chassis.updateOdo();
        valueTurnover.setCurrentPos(chassis.getPose());
        valueTurnover.setIsRed(true);
        commandScheduler.endAll();
    }

}
