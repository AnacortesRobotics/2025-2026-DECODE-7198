package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;

import java.util.Locale;

public class Chassis implements Subsystem {

    private PIDCoefficients pidCoefficients;
    private LinearTrajectory trajectory;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private double scaleSpeed = 1;
    private double maxSpeed = 1;

    private GoBildaPinpointDriver odo;
    private Telemetry telemetry;

    public PIDController pidForward = new PIDController(pidCoefficients.XP, pidCoefficients.XI, pidCoefficients.XD);
    public PIDController pidHorizontal = new PIDController(pidCoefficients.YP, pidCoefficients.YI, pidCoefficients.YD);
    public PIDController pidRotate = new PIDController(pidCoefficients.RP, pidCoefficients.RI, pidCoefficients.RD);

    public Chassis(HardwareMap hMap, Telemetry telemetry) {
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

        pidHorizontal.setInverted(true);

        this.telemetry = telemetry;

        updateOdo();
        trajectory = new LinearTrajectory(telemetry, odo.getPosition());
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
        double heading = -odo.getHeading(AngleUnit.RADIANS);
        double robotVert = Math.sin(heading) * horizontal + Math.cos(heading) * vertical;
        double robotHoriz = Math.cos(heading) * horizontal - Math.sin(heading) * vertical;
        mecanumDrive(robotVert, robotHoriz, rotate);
    }

    private void updateOdo() {
        odo.update();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", odo.getPosition().getX(DistanceUnit.INCH), odo.getPosition().getY(DistanceUnit.INCH), odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("True Position (Raw odo values)", data);
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

    private void setTarget(Pose2D target) {
        pidForward.setTarget(target.getX(DistanceUnit.INCH));
        pidHorizontal.setTarget(target.getY(DistanceUnit.INCH));
        pidRotate.setTarget(target.getHeading(AngleUnit.DEGREES));
    }

    private boolean isAtTarget() {
        return Math.abs(odo.getPosition().getX(DistanceUnit.INCH) - pidForward.getTarget()) < .5 &&
                Math.abs(odo.getPosition().getY(DistanceUnit.INCH) - pidHorizontal.getTarget()) < .5 &&
                Math.abs(odo.getPosition().getHeading(AngleUnit.DEGREES) - pidRotate.getTarget()) < 2;
    }

    private void update() {
        updateOdo();
        mecanumDriveFieldCentric(pidForward.update(odo.getPosition().getX(DistanceUnit.INCH)),
                pidHorizontal.update(odo.getPosition().getY(DistanceUnit.INCH)),
                pidRotate.update(odo.getPosition().getHeading(AngleUnit.DEGREES))
                );
    }

    private void updateTrajectory() {
        updateOdo();
        setTarget(trajectory.getLookaheadPoint(odo.getPosition(), 6));
        mecanumDriveFieldCentric(pidForward.update(odo.getPosition().getX(DistanceUnit.INCH)),
                pidHorizontal.update(odo.getPosition().getY(DistanceUnit.INCH)),
                pidRotate.update(odo.getPosition().getHeading(AngleUnit.DEGREES)));

    }

    public void setPath(Pose2D... pose) {
        trajectory = new LinearTrajectory(telemetry, pose);
    }

    public Command driveToPosition(Pose2D target) {
        return new FunctionalCommand(
                ()->setTarget(target),
                this::update,
                (interrupted)->stop(),
                this::isAtTarget,
                this).setName("Drive to position");
    }

    public Command driveTrajectory(Pose2D... path) {
        return new FunctionalCommand(
                ()-> setPath(path),
                this::updateTrajectory,
                (interrupted)->stop(),
                this::isAtTarget,
                this).setName("Drive trajectory");
    }

    public void stop() {
        pidForward.stop();
        pidHorizontal.stop();
        pidRotate.stop();
        mecanumDriveFieldCentric(0, 0, 0);
    }

    public void setPidCoefficients() {
        pidForward.updatePIDCoefficients(pidCoefficients.XP, pidCoefficients.XI, pidCoefficients.XD);
        pidHorizontal.updatePIDCoefficients(pidCoefficients.YP, pidCoefficients.YI, pidCoefficients.YD);
        pidRotate.updatePIDCoefficients(pidCoefficients.RP, pidCoefficients.RI, pidCoefficients.RD);
    }

    public void setMaxSpeed(double speed) {
        maxSpeed = speed;
    }

    public void updateTelemetry() {
        trajectory.updateTelemetry();
        telemetry.addData("Target point", pidForward.getTarget() + ", " + pidHorizontal.getTarget() + ", " + pidRotate.getTarget());

    }

}
