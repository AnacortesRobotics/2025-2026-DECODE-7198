package org.firstinspires.ftc.teamcode;

//import android.hardware.Sensor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Config.RobotPose;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;

import java.util.function.BooleanSupplier;

//@Disabled
@TeleOp
public class TestingOpMode extends OpMode {

    AnalogInput encoder;

    private DistanceSensor distance;
    private TriggerList triggerList;
//    private DigitalChannel touchSensor;
//    private CommandScheduler commandScheduler;
//    static final double MAX_POS = 1.0;
//    static final double MIN_POS = 0.0;
//    double  position = (MAX_POS - MIN_POS) / 2;
//    Servo tServo;
//    CRServo sServo;
    RevColorSensorV3 colorSensor;
    Chassis chassis;

    VisionLimelight limelight;
    LimelightArtifact limelightArtifact;
    CommandScheduler commandScheduler;
    ValueTurnover valueTurnover;
    double heading = 0;

    double forward = 0;
    double strafe = 0;
    double rotate = 0;

    boolean isRed = true;

    private double currentHeading() {
        this.heading = -chassis.getPose().getHeading(AngleUnit.DEGREES);
        return this.heading;
    }

    @Override
    public void init() {
        commandScheduler = CommandScheduler.getInstance();
        chassis = new Chassis(hardwareMap, telemetry, true);
        commandScheduler.init(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        valueTurnover = ValueTurnover.getInstance();
        isRed = valueTurnover.getIsRed();
        triggerList = TriggerList.getInstance();
        limelight = new VisionLimelight(hardwareMap, telemetry);
        limelight.update();
        chassis.updateOdo();
        Command startTracking = chassis.autoTurn(()->0.0, ()->0.0, currentHeading() - limelight.targetX);


        commandScheduler.getTrigger(GamepadInput.RIGHT_BUMPER, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(() -> limelight.setTarget(VisionLimelight.VisionTarget.GREEN_ARTIFACT))); // green
        commandScheduler.getTrigger(GamepadInput.LEFT_BUMPER, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(() -> limelight.setTarget(VisionLimelight.VisionTarget.PURPLE_ARTIFACT))); // purple

        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(startTracking.cancel());

        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onPressed(startTracking);
        commandScheduler.setDefaultCommands(new InstantCommand(()->
                chassis.mecanumDrive(forward, strafe, rotate), chassis));
    }

    @Override
    public void start() {
        chassis.setCurrentPose(valueTurnover.getCurrentPos());
        chassis.setHolonomicOffset(isRed ? -Math.PI / 2 : Math.PI / 2);
    }

    @Override
    public void loop() {
        limelight.update();
        limelight.updateTelemetry();
        chassis.updateOdo();

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;
//        telemetry.addData("angle offset", limelight.getAngleOffSet());

        commandScheduler.run();
        commandScheduler.updateTelemetry();
    }

    public void stop() {
        commandScheduler.endAll();
    }

}
