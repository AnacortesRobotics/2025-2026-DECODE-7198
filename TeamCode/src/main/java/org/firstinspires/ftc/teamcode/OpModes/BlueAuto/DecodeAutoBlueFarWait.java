package org.firstinspires.ftc.teamcode.OpModes.BlueAuto;

import android.graphics.HardwareRenderer;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Config.RobotPose;

import org.firstinspires.ftc.teamcode.Controllers.LinearTrajectory;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.ValueTurnover;

@Autonomous
public class DecodeAutoBlueFarWait extends OpMode {

    Chassis chassis;
    LinearTrajectory trajectory;
    Launcher launcher;
    Indexer indexer;
    RobotPose robotPose;
    CommandScheduler commandScheduler;
    ValueTurnover valueTurnover;
    Boolean isRed = false;

    private static final double CHASSIS_MAX_SPEED = 0.9; // Drive Fast in Auto
    private static final double LAUNCHER_RPM = 5300; // Spin launcher up to 5300 RPM
    private static final double LAUNCHER_POWER = 1.0; // Spin Launcher Fast in Auto
    private RobotPose setupPose;
    private RobotPose startPose;
    private RobotPose poseFarShoot;
    private RobotPose poseMiddleShoot;
    private RobotPose moveToEnd;
    private RobotPose moveToCollect1stcycle;
    private RobotPose moveToCollect1stcycle1stBall;
    private RobotPose moveToCollect1stcycle2ndBall;
    private RobotPose moveToCollect1stcycle3rdBall;
    private Command driveTo(RobotPose pose, String name) {
        return chassis.driveToPosition(pose)
                .setName(name)
                .setInterruptable(false);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        launcher.updateTelemetry();

        indexer = new Indexer(hardwareMap, telemetry);
//        robotPose = new RobotPose();
//        trajectory = new LinearTrajectory(telemetry, new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        commandScheduler = CommandScheduler.getInstance();
        valueTurnover = ValueTurnover.getInstance();
        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, -72 + RobotCoefficients.ROBOT_LENGTH_HALF, 24 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0));


        // 0,0 is the center of field RobotPose
        // These are currently hardcoded to Blue Side, have a RedSide and BlueSide version going forward
        setupPose = new RobotPose(-72, 24, 0, true);
        startPose = new RobotPose(-69, 25, 16, true);
        poseFarShoot = new RobotPose(-69, 25, 16.0, true);
        poseMiddleShoot = new RobotPose(20, 20, 43, true);
        moveToEnd = new RobotPose(-69, 50, 0, true);
        moveToCollect1stcycle = new RobotPose(-45, 36, 90, true);
        moveToCollect1stcycle1stBall = moveToCollect1stcycle.plusY(5);//change to a negative number for red auto
        moveToCollect1stcycle2ndBall = moveToCollect1stcycle.plusY(10);
        moveToCollect1stcycle3rdBall = moveToCollect1stcycle.plusY(15);

        commandScheduler.init(this);

        Command wait = new WaitCommand(500);
        Command intake1st = indexer.intakeSlot(RobotCoefficients.INTAKESLOT1);
        Command intake2nd = indexer.intakeSlot(RobotCoefficients.INTAKESLOT2);
        Command intake3rd = indexer.intakeSlot(RobotCoefficients.INTAKESLOT3);
        Command moveIntakeUp = indexer.intakeAndScan();

        // this sets power of launcher to 1.0, then calls launcher.start later? Is that why it doesn't stop?
        // removed from end of prepareLauncher
        // ...wait,  launcher.chargeLauncher(LAUNCHER_POWER)
        Command prepareLauncher = new SequentialCommandGroup(
                driveTo(startPose, "Starting Position"),
                indexer.fireSlot(RobotCoefficients.SHOOTSLOT1), wait);

        //( Need to improve code by a) testing if motor has reached target RPM before shooting,
        //( and b)
        Command launchBallsFar = new ParallelRaceCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        driveTo(poseFarShoot,"Turn To Shoot"),
                        wait,
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT1), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT3), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT2), indexer.pitchToLauncher(), wait
                ), new SequentialCommandGroup(
                launcher.setRPM(RobotCoefficients.LONG_RPM), launcher.runLauncher())).
                addRequirements(chassis).setName("Launch Balls Far").setInterruptable(false);

        Command launchBallsFar2 = new ParallelRaceCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        driveTo(poseFarShoot,"Turn To Shoot"),
                        wait,
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT1), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT3), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT2), indexer.pitchToLauncher(), wait//left off here feel tierd
                ), new SequentialCommandGroup(
                launcher.setRPM(RobotCoefficients.LONG_RPM), launcher.runLauncher())).
                addRequirements(chassis).setName("Launch Balls Far").setInterruptable(false);


        Command launchBallsMiddle = new ParallelRaceCommandGroup(
                new SequentialCommandGroup(
                        new WaitCommand(1000),
                        driveTo(poseMiddleShoot,"Turn To Shoot"), wait,
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT1), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT3), indexer.pitchToLauncher(), new WaitCommand(750),
                        indexer.fireSlot(RobotCoefficients.SHOOTSLOT2), indexer.pitchToLauncher(), wait
                ), new SequentialCommandGroup(
                launcher.setRPM(RobotCoefficients.SHORT_RPM), launcher.runLauncher())).
                addRequirements(chassis).setName("Launch Balls Middle").setInterruptable(false);


        Command intake1stCycle = new SequentialCommandGroup(
                driveTo(moveToCollect1stcycle, "1stCycle"), new InstantCommand(()->chassis.setMaxSpeed(.3)), wait,
                intake3rd, wait, driveTo(moveToCollect1stcycle1stBall, "1st Cycle 3 Ball") , wait, moveIntakeUp,
                intake1st, wait, driveTo(moveToCollect1stcycle2ndBall, "1st Cycle 1 Ball") , wait, moveIntakeUp,
                intake2nd, wait, driveTo(moveToCollect1stcycle3rdBall, "1st Cycle 2 Ball") , wait, moveIntakeUp,
                new InstantCommand(()->chassis.setMaxSpeed(.8))
        ).addRequirements(chassis).setName("Launch Balls").setInterruptable(false);

        Command goToEnd = new SequentialCommandGroup(driveTo(moveToEnd, "Ending Move"), launcher.setRPM(0)).addRequirements(chassis).setName("Ending").setInterruptable(false);

        commandScheduler.schedule(
                new SequentialCommandGroup(
//                       prepareLauncher,
                        launchBallsFar,
//                        intake1stCycle,
//                        launchBallsMiddle,
                        goToEnd
                        /*launcher.setRPM(RobotCoefficients.LONG_RPM), launcher.runLauncher(),*/
                        /*new InstantCommand(()->chassis.stop()*/
                ).setName("runLauncher"));
    }

    /**
     * Initializes the autonomous execution state.
     * <p>
     * This method performs the following setup sequence:
     * <ol>
     * <li><b>Alliance Detection:</b> Determines if the robot is on the Red alliance by
     * checking if the current X-coordinate is negative.</li>
     * <li><b>Global State Update:</b> Synchronizes the {@code valueTurnover} utility
     * with the detected alliance color.</li>
     * <li><b>Localization Reset:</b> Calibrates the chassis to the predefined
     * {@code startPose}.</li>
     * <li><b>Performance Tuning:</b> Sets the drivetrain to the maximum allowed
     * autonomous velocity.</li>
     * </ol>
     * * @see #CHASSIS_MAX_SPEED
     */
    @Override
    public void start() {
//        chassis.setCurrentPose(new Pose2D(DistanceUnit.INCH, -72 + RobotCoefficients.ROBOT_LENGTH_HALF, 24 - chassis.ROBOT_WIDTH / 2, AngleUnit.DEGREES, 0));
        // Determine alliance based on field side (X < 0 is Red in standard FTC coordinates)
        chassis.setCurrentPose(setupPose);
        Pose2D currentPose = chassis.getPose();
        // isRed = false;
        isRed = (currentPose.getY(DistanceUnit.INCH) < 0) ? true : false;
        // Pass the alliance state to the shared value container
        valueTurnover.setIsRed(isRed);

        // Apply the high-speed autonomous constant
        chassis.setMaxSpeed(CHASSIS_MAX_SPEED);

    }

    /**
     * Main execution loop for the autonomous OpMode.
     * <p>
     * This method is called repeatedly while the OpMode is active and performs the following:
     * <ul>
     * <li>Updates the chassis odometry to maintain accurate field localization.</li>
     * <li>Triggers the {@link CommandScheduler} to process and execute active commands.</li>
     * <li>Aggregates and pushes telemetry data to the Driver Station, including:
     * <ul>
     * <li>The detected alliance color (Red vs. Blue).</li>
     * <li>Active command status and subsystem-specific diagnostics (Chassis and Launcher).</li>
     * </ul>
     * </li>
     * </ul>
     */
    @Override
    public void loop() {
        chassis.updateOdo();

        commandScheduler.run();

        telemetry.addData("Alliance:", isRed ? "Red" : "Blue");
        commandScheduler.updateTelemetry();
        chassis.updateTelemetry();
        launcher.updateTelemetry();
    }

    /**
     * Handles the safe shutdown and state persistence of the robot when the OpMode is stopped.
     * <p>
     * This method performs the following cleanup operations:
     * <ul>
     * <li><b>Hardware Safety:</b> Immediately deactivates the launcher motors.</li>
     * <li><b>Final Localization:</b> Performs a final update of the odometry sensors to capture
     * the robot's exact ending position.</li>
     * <li><b>State Persistence:</b> Passes the final {@link RobotPose} to the {@code valueTurnover}
     * singleton, allowing the TeleOp OpMode to inherit the robot's coordinates for
     * field-centric driving.</li>
     * <li><b>Command Cleanup:</b> Forces the {@link CommandScheduler} to deschedule and
     * cleanly terminate all remaining active tasks.</li>
     * </ul>
     */
    @Override
    public void stop() {
        //launcher.stop();
        chassis.updateOdo();
        valueTurnover.setCurrentPos(chassis.getPose());
        valueTurnover.setIsRed(false);
        commandScheduler.endAll();
    }
}