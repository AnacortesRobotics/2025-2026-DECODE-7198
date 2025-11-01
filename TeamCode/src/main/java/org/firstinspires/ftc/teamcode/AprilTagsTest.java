package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;


@TeleOp
public class AprilTagsTest extends OpMode {
    public Telemetry telemetry;
    public Limelight3A limeLight;

    private CommandScheduler commandScheduler;

    public AprilTagsTest(HardwareMap hMap, Telemetry telemetry){
        limeLight = hMap.get(Limelight3A.class, "limeLight");
        telemetry.addData("hello World", "hello World");
    }
    @Override
    public void init() {
        commandScheduler = CommandScheduler.getInstance();

        commandScheduler.init(this);

        telemetry.addData("it works", "hello");
    }

    public void loop() {
        telemetry.addData("goodbye", "goodbye");

    }
}