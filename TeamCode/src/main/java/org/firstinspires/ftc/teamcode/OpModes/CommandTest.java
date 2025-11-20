package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.ValueTurnover;

import javax.xml.validation.Validator;

@Disabled
@Autonomous
public class CommandTest extends OpMode {

    private CommandScheduler commandScheduler;

    private Launcher launcher;
    private Chassis chassis;
    private double launcherPower = 0;

    private ValueTurnover valueTurnover;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);

        commandScheduler = CommandScheduler.getInstance();
        valueTurnover = ValueTurnover.getInstance();
        commandScheduler.init(this);

        commandScheduler.schedule(new ParallelCommandGroup(
                chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0)),
                //chassis.autoTurn(()->0, ()->0, new Pose2D(DistanceUnit.INCH, 6, 6, AngleUnit.DEGREES, 0))
                launcher.chargeLauncher(500)

        ));

//        commandScheduler.schedule(chassis.driveTrajectory(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
//                new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0),
//                new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 90),
//                new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 90)
//        ));
//        chassis.setMaxSpeed(.4);
    }

    @Override
    public void loop() {
        chassis.updateOdo();

        chassis.updateTelemetry();
        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

    @Override
    public void stop() {
        chassis.updateOdo();
        valueTurnover.setIsRed(false);
        valueTurnover.setCurrentPos(chassis.getPose());
        commandScheduler.endAll();
    }

}
