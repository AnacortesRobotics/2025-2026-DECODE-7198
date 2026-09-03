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
import org.firstinspires.ftc.teamcode.Controllers.LinearTrajectory;
import org.firstinspires.ftc.teamcode.Controllers.PIDController;

import java.util.Locale;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class WheelDropChassis implements Subsystem {

    private LinearTrajectory trajectory;

    public final double ROBOT_WIDTH = 15.625;
    public final double ROBOT_LENGTH = 15.25;

    public DcMotor left;
    public DcMotor right;

    private double scaleSpeed = 1;
    private double maxSpeed = 1;

    private GoBildaPinpointDriver odo;
    private Telemetry telemetry;

    private double holonomicOffset = 0;

    public PIDController pidForward = new PIDController(PIDCoefficients.XP, PIDCoefficients.XI, PIDCoefficients.XD, false);
    public PIDController pidHorizontal = new PIDController(PIDCoefficients.YP, PIDCoefficients.YI, PIDCoefficients.YD, false);
    public PIDController pidRotate = new PIDController(PIDCoefficients.RP, PIDCoefficients.RI, PIDCoefficients.RD, true);

    public WheelDropChassis(HardwareMap hMap, Telemetry telemetry, boolean useOdo) {

        left = hMap.get(DcMotor.class, "frontLeft");
        left.setDirection(DcMotor.Direction.REVERSE);
        right = hMap.get(DcMotor.class, "frontRight");
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void dropChassisDrive(double forward, double rotate){
        left.setPower(forward+rotate);
        right.setPower(forward-rotate);
    }

    public void setHolonomicOffset(double offsetPI) {
        holonomicOffset = offsetPI;
    }

    public void updateOdo() {
        odo.update();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", odo.getPosition().getX(DistanceUnit.INCH), odo.getPosition().getY(DistanceUnit.INCH), odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("True Position (Raw odo values)", data);
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

    private void update() {
        double lastOffset = holonomicOffset;
        holonomicOffset = 0;
        holonomicOffset = lastOffset;
    }

    private double getAngleFromPoint(Pose2D target) {
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


    public void stop() {
        pidForward.stop();
        pidHorizontal.stop();
        pidRotate.stop();
        dropChassisDrive(0, 0);
    }

    public void setPidCoefficients() {
        pidForward.updatePIDCoefficients(PIDCoefficients.XP, PIDCoefficients.XI, PIDCoefficients.XD);
        pidHorizontal.updatePIDCoefficients(PIDCoefficients.YP, PIDCoefficients.YI, PIDCoefficients.YD);
        pidRotate.updatePIDCoefficients(PIDCoefficients.RP, PIDCoefficients.RI, PIDCoefficients.RD);
    }

    public void setMaxSpeed(double speed) {
        maxSpeed = speed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void updateTelemetry() {
        telemetry.addData("Target point", pidForward.getTarget() + ", " + pidHorizontal.getTarget() + ", " + pidRotate.getTarget());
        telemetry.addData("Max speed", maxSpeed);
        trajectory.updateTelemetry();
    }
}
