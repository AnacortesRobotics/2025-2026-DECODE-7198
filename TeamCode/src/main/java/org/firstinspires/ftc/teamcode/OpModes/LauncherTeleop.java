package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;

@Disabled
@TeleOp
public class LauncherTeleop extends OpMode {
    private Chassis chassis;
    private CommandScheduler commandScheduler;
    private Launcher launcher;
    private final double RPM_INCREMENTS = 50;

    private RevColorSensorV3 colorSensor;

    private DigitalChannel magnet;

//    private CRServo axon;
//    private AnalogInput axonPos;
//
//    private double axonPose = 0;
//    private double lastPose = 0;

    @Override
    public void init() {

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSens");

        magnet = hardwareMap.get(DigitalChannelImpl.class, "magnet");

//        axon = hardwareMap.get(CRServo.class, "indexerServo");
//        axonPos = hardwareMap.get(AnalogInput.class, "indexerPOS");

//        launcher = new Launcher(hardwareMap, telemetry);
//
//        commandScheduler = CommandScheduler.getInstance();
////
//        commandScheduler.init(this);
//
//        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->axon.setPower(.2)));
//        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->axon.setPower(0));

        //        chassis = new Chassis(hardwareMap, telemetry, false);
//
//        commandScheduler
//                .getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY)
//                .onJustPressed(launcher.start());
//        commandScheduler
//                .getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY)
//                .onJustPressed(launcher.stop());
//        commandScheduler
//                .getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY)
//                .onJustPressed(launcher.adjustRPM(RPM_INCREMENTS));
//        commandScheduler
//                .getTrigger(GamepadInput.LEFT_BUMPER, GamepadIndex.PRIMARY)
//                .onJustPressed(launcher.adjustRPM(-RPM_INCREMENTS));
//        commandScheduler.setDefaultCommands(
//                new InstantCommand(()->
//                        chassis.mecanumDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x), chassis));
    }

    @Override
    public void loop() {
        //commandScheduler.run();

//        telemetry.addData("Red", colorSensor.red());
//        telemetry.addData("Green", colorSensor.green());
//        telemetry.addData("Blue", colorSensor.blue());

        telemetry.addData("magnet", magnet.getState());

//        if (lastPose > (axonPos.getVoltage() / axonPos.getMaxVoltage() * 180) + .3) {
//            axonPose += 180;
//        }
//        lastPose = axonPos.getVoltage() / axonPos.getMaxVoltage() * 180;
//        telemetry.addData("Axon pos", (axonPose + axonPos.getVoltage() / axonPos.getMaxVoltage() * 180));

//      telemetry.addData("target rpm", targetLauncherRpm);
//        telemetry.addData("left rpm", launcher.getCurrentRPM(Launcher.LauncherWheel.LEFT));
//        telemetry.addData("right rpm", launcher.getCurrentRPM(Launcher.LauncherWheel.RIGHT));
//        telemetry.addData("Is it working ", launcher.isSpinning());
//
//        commandScheduler.run();
//        commandScheduler.updateTelemetry();
    }

    @Override
    public void stop() {
        commandScheduler.endAll();
    }

}
