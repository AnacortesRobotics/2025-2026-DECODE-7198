package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous
@Disabled
public class AutoTest extends OpMode {

//    Chassis chassis;
//
//    private long timeToLeave = 0;
//
//    private boolean stage;
//
//    public static double xDistance = 16;
//    public static double yDistance = 0;
//    public static double rDistance = 0;
//
//    @Override
    public void init() {
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        chassis = new Chassis();
//
//        chassis.init(hardwareMap, telemetry);
//
//        chassis.setTarget(new Pose2D(DistanceUnit.INCH, xDistance, yDistance, AngleUnit.DEGREES, rDistance));
    }
//
//    @Override
    public void loop() {
//
//        if (System.currentTimeMillis() < timeToLeave) {
//            return;
//        }
//
//        if (gamepad1.a) {
//            chassis.setPidCoefficients();
//        }
//
//        if (!chassis.isAtTarget()) {
//            chassis.update();
//        } else {
//            chassis.stop();
//            timeToLeave = System.currentTimeMillis() + 1000;
//            if (stage) {
//                chassis.setTarget(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//            } else {
//                chassis.setTarget(new Pose2D(DistanceUnit.INCH, xDistance, yDistance, AngleUnit.DEGREES, rDistance));
//            }
//            stage = !stage;
//        }
//        chassis.updateOdo();
//
    }
}
