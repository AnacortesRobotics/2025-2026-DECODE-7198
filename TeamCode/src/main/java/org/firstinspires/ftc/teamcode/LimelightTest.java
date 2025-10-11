package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

@TeleOp
public class LimelightTest extends OpMode {

    //private Limelight3A limelight;

    private Chassis chassis;
    private LinearTrajectory linearTrajectory;

    private double forward;
    private double strafe;
    private double rotate;

    @Override
    public void init() {
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);

        chassis = new Chassis(hardwareMap, telemetry);
        linearTrajectory = new LinearTrajectory(telemetry,
                new Pose2D(DistanceUnit.INCH, -10, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 10, 0, AngleUnit.DEGREES, 0)
                );
    }

    @Override
    public void start() {
        //limelight.start();
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        chassis.mecanumDriveFieldCentric(forward, strafe, rotate);

        telemetry.addData("Target point", linearTrajectory.getLookaheadPoint(chassis.getPose(), 8));

//        LLResult result = limelight.getLatestResult();
//        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//        for (LLResultTypes.FiducialResult fr : fiducialResults) {
//            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
//        }
        telemetry.update();
    }

}
