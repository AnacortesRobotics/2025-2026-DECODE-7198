/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Subsystems.Chassis;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightArtifact;


/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "oblisk test", group = "Sensor")

public class VisionTest extends LinearOpMode {
    DcMotor leftdrive = null;
    DcMotor rightdrive = null;
    LimelightArtifact limelightInfo;

    private Servo servo = null;
    boolean last = false;
    boolean autoTurnState = false;
    private boolean purpleTracking = false;
    private boolean greenTracking = false;
    double turn;
    public LLResultTypes.ColorResult trackingResult;
    Chassis chassis;
    CommandScheduler commandScheduler;

    void drive (double forward,double turning)
    {
        leftdrive.setPower(forward-turning);
        rightdrive.setPower(forward+turning);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        commandScheduler = CommandScheduler.getInstance();

        limelightInfo  = new LimelightArtifact(hardwareMap, telemetry, 1);

//        limelight.setPipeline(1);

        telemetry.setMsTransmissionInterval(11);



        // List<LLResultTypes.FiducialResult> fiducialResults = new ArrayList<>();

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        leftdrive = hardwareMap.get(DcMotor.class, "leftdrive");
        leftdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightdrive = hardwareMap.get(DcMotor.class, "rightdrive");
        rightdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.init(this);



        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                autoTurnState = true;
            }
            if (gamepad1.b) {
                autoTurnState = false;
            }
            if (gamepad1.right_bumper) {
                telemetry.addData("right", "works");
                limelightInfo.changePipeline(1);

            }
            if (gamepad1.left_bumper) {
                telemetry.addData("left", "works");
                limelightInfo.changePipeline(2);

            }
            if (autoTurnState){
//                telemetry.addData("autoTurnState", "is true");
//                telemetry.addData("greenTracking", greenTracking);
//                telemetry.addData("purpleTracking", purpleTracking);
                trackingResult = limelightInfo.getColorTrackingResults();
                commandScheduler.schedule(chassis.autoTurn(()->0, ()->0, trackingResult.getTargetXDegrees()));
//                telemetry.addData("trackingResult", trackingResult);
//                if (trackingResult != null) {
//                    double trackingResultArea = trackingResult.getTargetArea();
////                double purpleArea = purpleColorResult.getTargetArea();
////                turn = 0;
//
//                    telemetry.addData("colorresult area", trackingResultArea);
//
//                    if (trackingResultArea > 0.0035) {
//                        telemetry.addData("colorresult status", "is > 0.0035");
//                        if (Math.abs(trackingResult.getTargetXDegrees()) > 7.5) {
//                            turn = .025 * (trackingResult.getTargetXDegrees()/*/Math.sqrt(area)*/);
//                            drive(gamepad1.left_stick_y, turn);
//                        }
////                    telemetry.addData("distance value", colorresult.);
//                    }
//                    else {
//                        drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
//                    }
                commandScheduler.run();
            }
//                if (colorresult.getTargetArea() > 2){
//
//                }

            }
//            else {
//                telemetry.addData("distance value", colorresult.getTargetArea());
                drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
//            }

            telemetry.update();
        }
//    }
}
