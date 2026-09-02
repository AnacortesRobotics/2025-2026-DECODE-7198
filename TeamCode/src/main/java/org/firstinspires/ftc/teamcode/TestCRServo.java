package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Thread.sleep;

public class TestCRServo implements Subsystem {
    private Telemetry telemetry;
    private CRServo indexer;
    private AnalogInput indexerPosition;
    private Direction direction;
    private double pastPos;
    private double nowPos;

    public enum Direction {
        FORWARD,
        REVERSE
    }

    public TestCRServo(HardwareMap hMap, Telemetry telemetry) {// the error is fine if no error delete this
        // Left and right from the servo side, not ramp side
        indexer = hMap.get(CRServo.class, "indexerServo");
        indexerPosition = hMap.get(AnalogInput.class, "indexerPOS");
        this.direction = Direction.FORWARD;
        this.telemetry = telemetry;
    }

    public Command turnSpindexer() {
        // need a target to set - add once this code is tested
        if (indexerPosition == null) {
            pastPos = 0;
        } else {
            pastPos = nowPos;
        }

        nowPos = getServoPosition();
        return new InstantCommand(
                () -> indexer.setPower(0.05)
        );
    }

    public Command stopTurningSpindexer() {
        if (nowPos == 0) {
            return new InstantCommand(
                    () -> indexer.setPower(0.05)
            );
        }
        return new InstantCommand(
                () -> indexer.setPower(0)
        );
    }

    public double getServoPosition() {
        return (indexerPosition.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
    }

    public void updateTelemetry() {
        telemetry.addData("y button past position:", pastPos);
        telemetry.addData("y button now position:", nowPos);
        telemetry.addData("getVoltage",indexerPosition.getVoltage());
    }
}