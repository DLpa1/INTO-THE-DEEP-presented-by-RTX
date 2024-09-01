package org.firstinspires.ftc.teamcode.Mock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static java.lang.Math.*;

@TeleOp
public class Teleop extends LinearOpMode {
    private DcMotorEx fl, fr, bl, br;


    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            teleLoop();
        }
    }

    public void teleLoop() {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        // [-1,1] -1 = 0.5
        // 50% = 0.5

        double drive_y = gamepad1.left_stick_y;
        double drive_x = -1.1*gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        frontLeftPower = drive_y + drive_x + turn;
        frontRightPower = drive_y - drive_x - turn;
        backLeftPower = drive_y - drive_x + turn;
        backRightPower = drive_y + drive_x - turn;

        double max = Math.max((drive_x + drive_y + turn), 1);

        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;

        fr.setPower(frontRightPower);
        fl.setPower(frontLeftPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);


        telemetry.addData("fl power", frontLeftPower);
        telemetry.addData("fr power", frontRightPower);
        telemetry.addData("bl power", backLeftPower);
        telemetry.addData("br power", backRightPower);
        telemetry.update();

    }
}
