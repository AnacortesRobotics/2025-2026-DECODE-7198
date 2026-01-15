package org.firstinspires.ftc.teamcode.OpModes.BlueAuto;

import com.qualcomm.hardware.ams.AMSColorSensor;
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
public class DecodeAutoBlueFarV2 extends OpMode {

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
//        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        valueTurnover = ValueTurnover.getInstance();
        commandScheduler.init(this);

        Command wait = new WaitCommand(500);
        Command turnToShoot = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -69 + RobotCoefficients.ROBOT_LENGTH_HALF, 25 - RobotCoefficients.ROBOT_WIDTH_HALF, AngleUnit.DEGREES, 15.75)).setName("Turn To Shoot").setInterruptable(false);
        Command moveToCollect1stcycle = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -45.5 + RobotCoefficients.ROBOT_LENGTH_HALF, 36 - RobotCoefficients.ROBOT_WIDTH_HALF, AngleUnit.DEGREES, 90)).setName("Move To 1st Collect").setInterruptable(false);
        Command moveToCollect1stcycle1stBall = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -45.5 + RobotCoefficients.ROBOT_LENGTH_HALF, 42 - RobotCoefficients.ROBOT_WIDTH_HALF, AngleUnit.DEGREES, 90)).setName("Move To 1st Collect").setInterruptable(false);
        Command moveToCollect1stcycle2ndBall = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -45 + RobotCoefficients.ROBOT_LENGTH_HALF, 47 - RobotCoefficients.ROBOT_WIDTH_HALF, AngleUnit.DEGREES, 90)).setName("Move To 2nd Collect").setInterruptable(false);
        Command moveToCollect1stcycle3rdBall = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -45 + RobotCoefficients.ROBOT_LENGTH_HALF, 52 - RobotCoefficients.ROBOT_WIDTH_HALF, AngleUnit.DEGREES, 90)).setName("Move To 3rd Collect").setInterruptable(false);

        Command moveIntakeUp = indexer.intakeAndScan();

        Command intake1st = indexer.intakeSlot(RobotCoefficients.SLOT1);
        Command intake2nd = indexer.intakeSlot(RobotCoefficients.SLOT2);
        Command intake3rd = indexer.intakeSlot(RobotCoefficients.SLOT3);
        Command prepareLauncher = new SequentialCommandGroup(indexer.fireSlot(RobotCoefficients.SLOT1), wait,  launcher.chargeLauncher(1));
        Command intake1stCycle = new SequentialCommandGroup(
//                launcher.chargeLauncher(0),
                launcher.stop(),
                moveToCollect1stcycle, new InstantCommand(()->chassis.setMaxSpeed(.3)), wait,
                intake1st, wait, moveToCollect1stcycle1stBall , wait, moveIntakeUp,
                intake2nd, wait, moveToCollect1stcycle2ndBall , wait, moveIntakeUp,
                intake3rd, wait, moveToCollect1stcycle3rdBall , wait, moveIntakeUp,
                new InstantCommand(()->chassis.setMaxSpeed(.8)));

        Command launchBalls = new ParallelRaceCommandGroup(new SequentialCommandGroup(
                turnToShoot,
                wait, indexer.fireSlot(RobotCoefficients.SLOT1), indexer.pitchToLauncher(), wait,
                indexer.fireSlot(RobotCoefficients.SLOT2), indexer.pitchToLauncher(), wait,
                indexer.fireSlot(RobotCoefficients.SLOT3), indexer.pitchToLauncher(), wait
//                launcher.stop(), new WaitCommand(1000)
                ),

                new SequentialCommandGroup(/*launcher.setRPM(5300),*/ launcher.start())).
                addRequirements(chassis).setName("Launch Balls").setInterruptable(false);

        Command moveToEnd = chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, -72 + RobotCoefficients.ROBOT_LENGTH_HALF, 48 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0)
        ).setName("Move to End");

        Command firstSegment = prepareLauncher;
        Command secondSegment = new ParallelRaceCommandGroup(launchBalls);
        Command thirdSegment = intake1stCycle;
//        Command fourthSegment = new ParallelCommandGroup(intake2nd, wait,  moveToCollect1stcycle2ndBall, wait, moveIntakeUp);
//        Command/*the end segment. implement at end*/ fourthSegment = new ParallelCommandGroup(new InstantCommand(()->chassis.setMaxSpeed(.8)), moveToEnd, launcher.stop());

        commandScheduler.schedule(new SequentialCommandGroup(
                firstSegment, secondSegment, thirdSegment, secondSegment, /*fourthSegment,*/ new InstantCommand(()->chassis.stop())
        ));
    }
//
    @Override
    public void start() {
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, -72 + RobotCoefficients.ROBOT_LENGTH_HALF, 24 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0));
        chassis.setMaxSpeed(.9);
        //launcher.setRPM(4800);
        commandScheduler.schedule(new RepeatCommand(new SequentialCommandGroup(chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)).setName("DriveFar"),
                chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)).setName("DriveBack"))));

    }
//
    @Override
    public void loop() {
        chassis.updateOdo();

        commandScheduler.run();

        commandScheduler.updateTelemetry();
        chassis.updateTelemetry();
        launcher.updateTelemetry();
    }
//
//    @Override
    public void stop() {
        launcher.stop();
        chassis.updateOdo();
        valueTurnover.setCurrentPos(chassis.getPose());
        valueTurnover.setIsRed(false);
        commandScheduler.endAll();
    }

}
