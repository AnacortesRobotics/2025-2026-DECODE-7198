package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;

@Autonomous
public class CommandTest extends OpMode {

    private CommandScheduler commandScheduler;

    private LauncherOld launcher;
    private Chassis chassis;
    private double launcherPower = 0;

    @Override
    public void init() {
        chassis = new Chassis(hardwareMap, telemetry, true);

        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);

        commandScheduler.schedule(chassis.driveTrajectory(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 90)
        ));
        chassis.setMaxSpeed(.4);
    }

    @Override
    public void loop() {

        chassis.updateTelemetry();
        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }

}
