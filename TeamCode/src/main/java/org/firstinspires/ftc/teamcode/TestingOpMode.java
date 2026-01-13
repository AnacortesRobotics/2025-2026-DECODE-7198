package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
//import android.hardware.Sensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;

//@Disabled
@TeleOp
public class TestingOpMode extends OpMode {

    AnalogInput encoder;

    private DistanceSensor distance;
//    private DigitalChannel touchSensor;
//    private CommandScheduler commandScheduler;
//    static final double MAX_POS = 1.0;
//    static final double MIN_POS = 0.0;
//    double  position = (MAX_POS - MIN_POS) / 2;
//    Servo tServo;
//    CRServo sServo;
    RevColorSensorV3 colorSensor;
    Chassis chassis;
    Launcher launcher;
    Indexer indexer;
    Limelight limelight;
    LimelightArtifact limelightArtifact;
    CommandScheduler commandScheduler;

    double forward = 0;
    double strafe = 0;
    double rotate = 0;


    @Override
    public void init() {
        commandScheduler = CommandScheduler.getInstance();
        chassis = new Chassis(hardwareMap, telemetry, true);
        launcher = new Launcher(hardwareMap, telemetry);
        indexer = new Indexer(hardwareMap, telemetry);
        limelightArtifact = new LimelightArtifact(hardwareMap, telemetry, 0);
        commandScheduler.init(this);

        launcher.stop();
        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                indexer.intakeMode(), indexer.intakeOpen()));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                indexer.shootingMode(), indexer.fireInOrder()));
        commandScheduler.getTrigger(GamepadInput.DPAD_RIGHT, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                indexer.shootingMode(), chassis.autoTurn(()->0.0, ()->0.0, limelightArtifact.getAngleOffSet(true))));
        commandScheduler.getTrigger(GamepadInput.DPAD_LEFT, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
                indexer.shootingMode(), chassis.autoTurn(()->0.0, ()->0.0, limelightArtifact.getAngleOffSet(false))));
        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.SECONDARY).onJustPressed(indexer.fireSlot(RobotCoefficients.SLOT1));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.SECONDARY).onJustPressed(indexer.fireSlot(RobotCoefficients.SLOT2));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.SECONDARY).onJustPressed(indexer.fireSlot(RobotCoefficients.SLOT3));
        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.SECONDARY).onJustPressed(indexer.pitchToLauncher());
        commandScheduler.getTrigger(GamepadInput.DPAD_DOWN, GamepadIndex.SECONDARY).onJustPressed(new InstantCommand(()->launcher.setPower(1)));
        commandScheduler.getTrigger(GamepadInput.DPAD_LEFT, GamepadIndex.SECONDARY).onJustPressed(new InstantCommand(()->launcher.setPower(.9)));
        commandScheduler.getTrigger(GamepadInput.DPAD_RIGHT, GamepadIndex.SECONDARY).onJustPressed(new InstantCommand(()->launcher.setPower(.9)));
        commandScheduler.getTrigger(GamepadInput.DPAD_UP, GamepadIndex.SECONDARY).onJustPressed(new InstantCommand(()->launcher.setPower(.8)));

        commandScheduler.getTrigger(GamepadInput.BACK_BUTTON, GamepadIndex.SECONDARY).onJustPressed(launcher.stop());
//        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.SECONDARY).onJustPressed(new SequentialCommandGroup(
//                indexer.intakeMode(), indexer.fixIntake()
//        ));
//        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(indexer.startIntake());
//        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(indexer.reverseIntake());
//        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(indexer.stopIntake());
        commandScheduler.getTrigger(GamepadInput.DPAD_DOWN, GamepadIndex.PRIMARY).onJustPressed(indexer.intakeSlot(RobotCoefficients.SLOT1));
        commandScheduler.getTrigger(GamepadInput.DPAD_LEFT, GamepadIndex.PRIMARY).onJustPressed(indexer.intakeSlot(RobotCoefficients.SLOT2));
        commandScheduler.getTrigger(GamepadInput.DPAD_RIGHT, GamepadIndex.PRIMARY).onJustPressed(indexer.intakeSlot(RobotCoefficients.SLOT3));
        commandScheduler.getTrigger(GamepadInput.DPAD_UP, GamepadIndex.PRIMARY).onJustPressed(indexer.intakeAndScan());
        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onPressed(chassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 12, 12, AngleUnit.DEGREES, 0)));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.stop()));
//        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.leftFront.setPower(1.0))).onJustReleased(new InstantCommand(()->chassis.leftFront.setPower(0.0)));
//        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.rightFront.setPower(1.0))).onJustReleased(new InstantCommand(()->chassis.rightFront.setPower(0.0)));
//        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.rightBack.setPower(1.0))).onJustReleased(new InstantCommand(()->chassis.rightBack.setPower(0.0)));
//        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->chassis.leftBack.setPower(1.0))).onJustReleased(new InstantCommand(()->chassis.leftBack.setPower(0.0)));

//        commandScheduler.getTrigger(GamepadInput.START_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new SequentialCommandGroup(launcher.setRPM(1000), launcher.start()));

        commandScheduler.setDefaultCommands(new InstantCommand(()->
                chassis.mecanumDrive(forward, strafe, rotate)));
//
//        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSens");
//
//        touchSensor.setMode(DigitalChannel.Mode.INPUT);
//
//
//        distance = hardwareMap.get(DistanceSensor.class, "distanceSens");
//        tServo = hardwareMap.get(Servo.class, "testServo");
//        sServo = hardwareMap.get(CRServo.class, "crServo");

    }

    @Override
    public void loop() {

        chassis.updateOdo();

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        commandScheduler.run();

        launcher.updateTelemetry();
        indexer.updateTelemetry();
        commandScheduler.updateTelemetry();

//        telemetry.addData("pos", encoder.getVoltage() * (360 / encoder.getMaxVoltage()));

//
//        if (!touchSensor.getState()) {
//            telemetry.addData("Button", "PRESSED");
//        } else {
//            telemetry.addData("Button", "NOT PRESSED");
//        }
//        if (gamepad1.aWasPressed()){
//            sServo.setPower(.5);
//        }
//        if (gamepad1.bWasPressed()){
//            sServo.setPower(-.5);
//        }

//        final float[] hsvValues = new float[3];
//
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//
//        Color.colorToHSV(colors.toColor(), hsvValues);
//
//        if (colors.green>0.015 && colors.green> colors.blue){
//            sServo.setPower((colors.green - colors.blue)*10);
//        }
//        else if(colors.blue>0.015 && colors.blue> colors.green){
//            sServo.setPower(-(colors.blue - colors.green)*10);
//        }
//        else {
//            sServo.setPower(0);
//        }
//
//        telemetry.addLine()
//                .addData("Red", colorSensor.red())
//                .addData("Green", colorSensor.green())
//                .addData("Blue", colorSensor.blue());
//
//        telemetry.addLine()
//                .addData("Hue", "%.3f", hsvValues[0])
//                .addData("Saturation", "%.3f", hsvValues[1])
//                .addData("Value", "%.3f", hsvValues[2]);
//        telemetry.addData("Alpha", "%.3f", colors.alpha);
//
//        telemetry.addData("servo position", tServo.getPosition());
//        telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
//        commandScheduler.run();
//        commandScheduler.updateTelemetry();


    }

    public void stop() {
        commandScheduler.endAll();
    }

}
