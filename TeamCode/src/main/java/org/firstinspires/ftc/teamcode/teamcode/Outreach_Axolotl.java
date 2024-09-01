/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Outreach_Axolotl")
//@Disabled
//
public class Outreach_Axolotl extends OpMode {
    // Declare OpMode members.

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo tailServo;



    @Override
    public void init() {

        //SET UP THE TAIL
        tailServo = hardwareMap.get(Servo.class, "tail");
        tailServo.setDirection(Servo.Direction.FORWARD);


        telemetry.addData("Tail position", tailServo.getPosition());
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD); //correct
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        tailServo.setPosition(0);
        // Tell the driver that initialization is complete.
    }


    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry

        double leftMotorPower;
        double rightMotorPower;


        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftMotorPower = Range.clip(drive + turn, -1.0, 1.0);
        rightMotorPower = Range.clip(drive - turn, -1.0, 1.0);



        // Send calculated power to wheels
        leftDrive.setPower(leftMotorPower);
        rightDrive.setPower(rightMotorPower);


//        //move to the right
//        if(gamepad1.dpad_right) {
//            if(tailServo.getPosition() != 0.2) {
//                do {
//                    sleep(100);
//                    tsPosition += 0.01;
//                    tailServo.setPosition(tsPosition);
//                } while (tailServo.getPosition() < 0.2);
//            }
//        }
//
//
//        // more to the left
//        if(gamepad1.dpad_left){
//            if(tailServo.getPosition() != 0.05){
//                do {
//                    sleep(100);
//                    tsPosition -= 0.01;
//                    tailServo.setPosition(tsPosition);
//                } while (tailServo.getPosition() > 0.05);
//            }
//
//        }

        //move to the right
        if (gamepad1.dpad_right) {
            tailServo.setPosition(tailServo.getPosition()+0.01);
        }
        //move to the left
        if (gamepad1.dpad_left) {
            tailServo.setPosition(tailServo.getPosition()-0.01);
        }


        telemetry.addData("Tail position", tailServo.getPosition());
//
//        // sleep the program
//        if(gamepad1.right_bumper){
//            sleep(1000);
//            tailServo.setPosition(0.2);
//        }
    }

    @Override
    public void stop() {
    }

    // write a method that can stall the program for a specified amount of time


    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }



}
