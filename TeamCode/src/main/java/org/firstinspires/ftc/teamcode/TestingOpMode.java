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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Config.RobotCoefficients;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Launcher;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadInput;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler.GamepadIndex;

import java.util.function.BooleanSupplier;

@TeleOp
public class TestingOpMode extends OpMode {

    private TouchSensor touch;
    private DcMotorEx motor;
    private RevColorSensorV3 color;

    private CommandScheduler commandScheduler;
    private TriggerList triggers;

    private BooleanSupplier one;
    private BooleanSupplier two;
    private BooleanSupplier three;
    private BooleanSupplier four;

    @Override
    public void init() {
        commandScheduler = CommandScheduler.getInstance();
        triggers = TriggerList.getInstance();
        commandScheduler.init(this);
        touch = hardwareMap.get(TouchSensor.class, "touchSens");
        motor = hardwareMap.get(DcMotorEx.class, "testMotorLuke");
        color = hardwareMap.get(RevColorSensorV3.class, "colorSens");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        one = ()->touch.isPressed();
        two = ()->motor.getCurrentPosition() > 400;

        commandScheduler.getTrigger(GamepadInput.A_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()-> {
            TriggerList triggerList;
            triggerList = TriggerList.getInstance();
            triggerList.addTrigger(one).onJustPressed(new InstantCommand(()->motor.setPower(.4)));
        }));
        commandScheduler.getTrigger(GamepadInput.B_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()-> {
            TriggerList triggerList;
            triggerList = TriggerList.getInstance();
            triggerList.removeTrigger(one);
        }));

        commandScheduler.getTrigger(GamepadInput.DPAD_DOWN, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()-> {
            TriggerList triggerList;
            triggerList = TriggerList.getInstance();
            triggerList.addTrigger(two).onJustPressed(new InstantCommand(()->motor.setPower(-.4)));
        }));
        commandScheduler.getTrigger(GamepadInput.DPAD_UP, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()-> {
            TriggerList triggerList;
            triggerList = TriggerList.getInstance();
            triggerList.removeTrigger(two);
        }));

        commandScheduler.getTrigger(GamepadInput.X_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->motor.setPower(-.4)));
        commandScheduler.getTrigger(GamepadInput.Y_BUTTON, GamepadIndex.PRIMARY).onJustPressed(new InstantCommand(()->motor.setPower(0)));



    }

    @Override
    public void loop() {
        commandScheduler.run();
        commandScheduler.updateTelemetry();
        telemetry.addData("touch", touch.isPressed());
        telemetry.addData("pos", motor.getCurrentPosition());
    }

    @Override
    public void stop() {
        triggers.stop();
    }

}
