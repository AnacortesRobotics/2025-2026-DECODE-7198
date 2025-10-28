package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.FunctionalCommand;
import org.firstinspires.ftc.teamcode.Commands.InstantCommand;
import org.firstinspires.ftc.teamcode.Commands.Subsystem;
import org.firstinspires.ftc.teamcode.Config.PIDCoefficients;
import org.firstinspires.ftc.teamcode.LinearTrajectory;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.Locale;
import java.util.function.DoubleSupplier;

public class Chassis implements Subsystem {

    private LinearTrajectory trajectory;

    public final double ROBOT_WIDTH = 15.625;
    public final double ROBOT_LENGTH = 15.25;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private double scaleSpeed = 1;
    private double maxSpeed = 1;

    private GoBildaPinpointDriver odo;
    private Telemetry telemetry;

    public PIDController pidForward = new PIDController(PIDCoefficients.XP, PIDCoefficients.XI, PIDCoefficients.XD, false);
    public PIDController pidHorizontal = new PIDController(PIDCoefficients.YP, PIDCoefficients.YI, PIDCoefficients.YD, false);
    public PIDController pidRotate = new PIDController(PIDCoefficients.RP, PIDCoefficients.RI, PIDCoefficients.RD, true);

    public Chassis(HardwareMap hMap, Telemetry telemetry, boolean useOdo) {

        leftFront = hMap.get(DcMotor.class, "frontLeft");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront = hMap.get(DcMotor.class, "frontRight");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack = hMap.get(DcMotor.class, "backLeft");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack = hMap.get(DcMotor.class, "backRight");
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (useOdo) {
            odo = hMap.get(GoBildaPinpointDriver.class, "odo");
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.setOffsets(3.14961, -5.94488, DistanceUnit.INCH);
//    x  11.151  half   5.575     y  13.375  half  6.6875
            odo.resetPosAndIMU();

        }
        pidHorizontal.setInverted(true);

        this.telemetry = telemetry;

        updateOdo();
        trajectory = new LinearTrajectory(telemetry, odo.getPosition());
    }

    public void setCurrentPose(Pose2D pose) {
        odo.setPosition(pose);
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

    public void updateOdo() {
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
        mecanumDriveFieldCentric(pidForward.update(odo.getPosition().getX(DistanceUnit.INCH)),
                pidHorizontal.update(odo.getPosition().getY(DistanceUnit.INCH)),
                pidRotate.update(odo.getPosition().getHeading(AngleUnit.DEGREES))
                );
    }

    private void updateTrajectory() {
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

    private double getAnglefromPoint(Pose2D target) {
        double xDif = target.getX(DistanceUnit.INCH) - odo.getPosX(DistanceUnit.INCH);
        double yDif = target.getY(DistanceUnit.INCH) - odo.getPosY(DistanceUnit.INCH);
        double targetAngle = Math.atan2(yDif, xDif) * 180 / Math.PI;
        if (targetAngle > 180) {
            targetAngle -= 360;
        } else if (targetAngle <= -180) {
            targetAngle += 360;
        }
        return targetAngle;
    }

    public Command autoTurn(DoubleSupplier forward, DoubleSupplier strafe, Pose2D target) {
        return new FunctionalCommand(()->{},
        ()->{
            double targetAngle = getAnglefromPoint(target);
            if (targetAngle - odo.getHeading(AngleUnit.DEGREES) > 180) {
                targetAngle -= 360;
            } else if (targetAngle - odo.getHeading(AngleUnit.DEGREES) < -180) {
                targetAngle += 360;
            }
            pidRotate.setTarget(targetAngle);
            mecanumDriveFieldCentric(forward.getAsDouble(), strafe.getAsDouble(),
                 pidRotate.update(odo.getHeading(AngleUnit.DEGREES)));
            },
            (interrupted)->{},
            ()->false,
            this).setInterruptable(true);
    }

    public void stop() {
        pidForward.stop();
        pidHorizontal.stop();
        pidRotate.stop();
        mecanumDriveFieldCentric(0, 0, 0);
    }

    public void setPidCoefficients() {
        pidForward.updatePIDCoefficients(PIDCoefficients.XP, PIDCoefficients.XI, PIDCoefficients.XD);
        pidHorizontal.updatePIDCoefficients(PIDCoefficients.YP, PIDCoefficients.YI, PIDCoefficients.YD);
        pidRotate.updatePIDCoefficients(PIDCoefficients.RP, PIDCoefficients.RI, PIDCoefficients.RD);
    }

    public void setMaxSpeed(double speed) {
        maxSpeed = speed;
    }

    public void updateTelemetry() {
        trajectory.updateTelemetry();
        telemetry.addData("Target point", pidForward.getTarget() + ", " + pidHorizontal.getTarget() + ", " + pidRotate.getTarget());

    }

}
