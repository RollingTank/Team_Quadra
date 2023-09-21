package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Testing")
public class Main extends OpMode {


    DcMotor RB;
    DcMotor LB;
    DcMotor RF;
    DcMotor LF;






    @Override
    public void init() {

        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");


    }

    public void loop() {

        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LF.setPower(0);


        if (gamepad1.left_stick_y < 0) {
            LB.setPower(gamepad1.left_stick_y);
            LF.setPower(gamepad1.left_stick_y);
        }
        if (gamepad1.left_stick_y > 0) {
            LB.setPower(gamepad1.left_stick_y);
            LF.setPower(gamepad1.left_stick_y);
        }

        if (gamepad1.right_stick_y > 0) {
            RB.setPower(-gamepad1.right_stick_y);
            RF.setPower(-gamepad1.right_stick_y);
        }
        if (gamepad1.right_stick_y < 0) {
            RB.setPower(-gamepad1.right_stick_y);
            RF.setPower(-gamepad1.right_stick_y);
        }


        if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0) {
            LB.setPower(-(gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            LF.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            RB.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            RF.setPower(-(gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        }

        if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0) {
            LB.setPower(-(gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            LF.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            RB.setPower((gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
            RF.setPower(-(gamepad1.left_stick_x + gamepad1.right_stick_x)/2);
        }



    }






}