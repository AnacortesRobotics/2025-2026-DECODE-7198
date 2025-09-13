package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class Chassis {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private double scaleSpeed = 1;
    private double maxSpeed = 1;

    private GoBildaPinpointDriver odo;
    private Telemetry telemetry;

    public void init(HardwareMap hMap, Telemetry telemetry) {
        leftFront = hMap.get(DcMotor.class, "frontLeft");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hMap.get(DcMotor.class, "frontRight");
        leftBack = hMap.get(DcMotor.class, "backLeft");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hMap.get(DcMotor.class, "backRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(461/64.0, 147/32.0, DistanceUnit.INCH);

        odo.resetPosAndIMU();

        this.telemetry = telemetry;
    }

    public void mecanumDrive(double forward, double strafe, double rotate) {
        double lfPower = (forward + strafe - rotate) * scaleSpeed;
        double rfPower = (forward - strafe + rotate) * scaleSpeed;
        double lbPower = (forward - strafe - rotate) * scaleSpeed;
        double rbPower = (forward + strafe + rotate) * scaleSpeed;
        double fastMotor = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower),
                        Math.max(Math.abs(lbPower),
                                Math.abs(rbPower))));
        double scaleFactor = Math.min(maxSpeed / fastMotor, 1);
        leftFront.setPower(lfPower * scaleFactor);
        rightFront.setPower(rfPower * scaleFactor);
        leftBack.setPower(lbPower * scaleFactor);
        rightBack.setPower(rbPower * scaleFactor);
    }

    public void mecanumDriveFieldCentric(double vertical, double horizontal, double rotate) {
        double heading = -odo.getHeading(AngleUnit.DEGREES);
        double robotVert = Math.sin(heading) * horizontal + Math.cos(heading) * vertical;
        double robotHoriz = Math.cos(heading) * horizontal - Math.sin(heading) * vertical;
        mecanumDrive(robotVert, robotHoriz, rotate);
    }

    public void updateOdo() {
        odo.update();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", odo.getPosition().getX(DistanceUnit.INCH), odo.getPosition().getY(DistanceUnit.INCH), odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("True Position (Raw odo values)", data);
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

}
