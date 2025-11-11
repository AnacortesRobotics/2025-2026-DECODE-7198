package org.firstinspires.ftc.teamcode.OpModes.BlueAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Controllers.LinearTrajectory;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.ValueTurnover;

@Autonomous
public class DecodeAutoBlueDepot extends OpMode {

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

        Command firstDriveSegment = chassis.driveTrajectory(
                new Pose2D(DistanceUnit.INCH, 72.3 - chassis.ROBOT_LENGTH / 2,  47 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, -175),
                new Pose2D(DistanceUnit.INCH, 12, 14, AngleUnit.DEGREES, 55)).setName("First Drive Segment");
        Command prepareLauncher = new SequentialCommandGroup(launcher.setRPM(4910), launcher.start());
        Command launchBalls = new SequentialCommandGroup(
                new RepeatCommand(
                        new SequentialCommandGroup(indexer.fireBall(), new WaitCommand(1500)), 4)).
                addRequirements(chassis).setName("Launch Balls").setInterruptable(false);
        Command moveToEnd = chassis.driveTrajectory(
                new Pose2D(DistanceUnit.INCH, 12, 14, AngleUnit.DEGREES, 55),
                new Pose2D(DistanceUnit.INCH,
                        70 - chassis.ROBOT_LENGTH / 2,
                        47 - chassis.ROBOT_WIDTH / 2,
                        AngleUnit.DEGREES, -179)
        ).setName("Move to End");

        Command firstSegment = new ParallelCommandGroup(firstDriveSegment, prepareLauncher);
        Command secondSegment = new ParallelCommandGroup(launchBalls, new InstantCommand(()->chassis.setMaxSpeed(.4)));
        Command thirdSegment = new ParallelCommandGroup(moveToEnd, launcher.stop());

        commandScheduler.schedule(firstSegment, secondSegment, thirdSegment,
                new InstantCommand(()-> chassis.stop()));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, 72.3 - chassis.ROBOT_LENGTH / 2.0, 47 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, -175));

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
        valueTurnover.setIsRed(false);
        commandScheduler.endAll();
    }

}
