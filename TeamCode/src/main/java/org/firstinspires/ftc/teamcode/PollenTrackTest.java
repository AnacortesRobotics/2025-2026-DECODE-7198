package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.VisionLimelight;

@TeleOp
public class PollenTrackTest extends OpMode {
    VisionLimelight limelight;

    @Override
    public void init() {
        limelight = new VisionLimelight(hardwareMap, telemetry);
        limelight.update();
        limelight.setPipeline(3);
    }

    @Override
    public void loop() {
        limelight.updateTelemetry();
    }
}
