package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class AutoTest extends OpMode {

    Chassis chassis;

    public PIDController pidForward = new PIDController(.1, 0, 0);
    public PIDController pidHorizontal = new PIDController(.1, 0, 0);
    public PIDController pidRotate = new PIDController(.01, 0, 0);

    @Override
    public void init() {
        chassis = new Chassis();
    }

    @Override
    public void loop() {



    }
}
