package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

@Autonomous
public class DecodeAuto extends OpMode {

    Chassis chassis;
    LinearTrajectory trajectory;
    Launcher launcher;
    Indexer indexer;
    CommandScheduler commandScheduler;

    // 0,0 is the loading zone in line with the red goal
    // or bottom right of F1 is the competition manual  (section 9.4)

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);

        Command firstDriveSegment = chassis.driveTrajectory(
                new Pose2D(DistanceUnit.INCH, 72 - chassis.ROBOT_LENGTH / 2,  24 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, -179),
                new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, -55)).setName("First Drive Segment");
        Command prepareLauncher = launcher.setRPM(4700).setName("Prepare Launcher");
        Command launchBalls = new SequentialCommandGroup(
                //new WaitCommand(800),
                new RepeatCommand(
                new SequentialCommandGroup(indexer.fireBall(), new WaitCommand(1000)), 4)).
                addRequirements(chassis).setName("Launch Balls");
        Command moveToEnd = chassis.driveTrajectory(
                new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, -55),
                new Pose2D(DistanceUnit.INCH,
                        70 - chassis.ROBOT_LENGTH / 2,
                        24 - chassis.ROBOT_WIDTH / 2,
                        AngleUnit.DEGREES, -179)
        ).setName("Move to End");
        Command testMove = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 16, 16, AngleUnit.DEGREES, -90));
        Command testMove2 = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 16, 16, AngleUnit.DEGREES, -179));

        commandScheduler.schedule(firstDriveSegment, launcher.start(), prepareLauncher, launchBalls, moveToEnd,
                new InstantCommand(()-> chassis.stop()),
                new InstantCommand(this::terminateOpModeNow).addRequirements(chassis));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, 72 - chassis.ROBOT_LENGTH / 2.0, 24 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, -179));
        //chassis.setMaxSpeed(.9);
        launcher.setRPM(4800);
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
        commandScheduler.endAll();
    }

}
