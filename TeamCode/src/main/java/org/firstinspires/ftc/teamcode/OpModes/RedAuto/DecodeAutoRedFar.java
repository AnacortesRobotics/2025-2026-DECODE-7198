package org.firstinspires.ftc.teamcode.OpModes.RedAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Controllers.LinearTrajectory;
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
    SequentialCommandGroup fullAuto;

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
                                                                                        //-61.375                            -17.1875
        Command turnToShoot = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -69 + chassis.ROBOT_LENGTH / 2.0, -25 + chassis.ROBOT_WIDTH / 2.0, AngleUnit.DEGREES, -21)).setName("Turn To Position");
        Command prepareLauncher = new SequentialCommandGroup(launcher.setRPM(RobotCoefficients.LONG_RPM), launcher.start());
        Command launchBalls = new RepeatCommand(
                        new SequentialCommandGroup(indexer.fireBall(), new WaitCommand(1500)), 4).
                addRequirements(chassis).setName("Launch Balls").setInterruptable(false);
        Command moveToEnd = chassis.driveToPosition(
                new Pose2D(DistanceUnit.INCH, -72 + chassis.ROBOT_LENGTH / 2.0, -48 + chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0)
        ).setName("Move to End");

        Command firstSegment = new ParallelRaceCommandGroup(turnToShoot, prepareLauncher).setName("First segment");
        Command secondSegment = new ParallelRaceCommandGroup(launchBalls, launcher.start()).setName("Second segment");
        Command thirdSegment = new ParallelCommandGroup(new InstantCommand(()->chassis.setMaxSpeed(.8)), launcher.stop(), moveToEnd).setName("Last segment");

        commandScheduler.schedule(new SequentialCommandGroup(
                firstSegment, secondSegment, thirdSegment, new InstantCommand(()->chassis.stop())
        ));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, -72 + chassis.ROBOT_LENGTH / 2.0, -24 + chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0));

    }

    @Override
    public void loop() {
        chassis.updateOdo();

        commandScheduler.run();

//        telemetry.addData("Sequence index", fullAuto.commandIndex);
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
