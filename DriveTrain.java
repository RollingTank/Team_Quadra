package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MainConfig")
public class DriveTrain extends OpMode {

    DcMotor RB;
    DcMotor LB;
    DcMotor RF;
    DcMotor LF;
    DcMotor Arm;

    DistanceSensor distanceSensor;
    Servo angleServo;
    Servo rightServo;
    Servo leftServo;
    Servo planeServo;

    double servoAngle;






    @Override
    public void init() {

        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo = hardwareMap.get(Servo.class, "servo1");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        planeServo = hardwareMap.get(Servo.class, "servo4");

        //servoAngle = 0.5;

        angleServo.setPosition(0.15);
        rightServo.setPosition(0.3);
        leftServo.setPosition(0.3);
        planeServo.setPosition(0);


    }


    public void loop() {

        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();
        //RB.setPower(-gamepad1.left_stick_y);
        //LB.setPower(gamepad1.right_stick_y);
        //RF.setPower(gamepad1.left_stick_y);
        //LF.setPower(-gamepad1.right_stick_y);


        RB.setPower(gamepad1.left_stick_y*0.75);
        LB.setPower(-gamepad1.right_stick_y*0.75);
        RF.setPower(gamepad1.left_stick_y*0.75);
        LF.setPower(-gamepad1.right_stick_y*0.75);

        //left
        if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0){
            LB.setPower(gamepad1.right_stick_x*0.75);
            LF.setPower(-gamepad1.right_stick_x*0.75);
            RB.setPower(gamepad1.left_stick_x*0.75);
            RF.setPower(-gamepad1.left_stick_x*0.75);
        }

        //right
        if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0){
            LB.setPower(gamepad1.right_stick_x*0.75);
            LF.setPower(-gamepad1.right_stick_x*0.75);
            RB.setPower(gamepad1.left_stick_x*0.75);
            RF.setPower(-gamepad1.left_stick_x*0.75);
        }

        //dpad movements
        if (gamepad1.dpad_right){
            LB.setPower(0.25);
            LF.setPower(-0.25);
            RB.setPower(0.3);
            RF.setPower(-0.25);
        }
        if (gamepad1.dpad_left){
            LB.setPower(-0.25);
            LF.setPower(0.25);
            RB.setPower(-0.3);
            RF.setPower(0.25);
        }
        if (gamepad1.dpad_up) {
            RB.setPower(0.2);
            LB.setPower(-0.2);
            RF.setPower(0.2);
            LF.setPower(-0.2);
        }
        if (gamepad1.dpad_down) {
            RB.setPower(-0.2);
            LB.setPower(0.2);
            RF.setPower(-0.2);
            LF.setPower(0.2);
        }

        //rotations
        if (gamepad1.left_trigger>0) {
            RB.setPower(-0.4);
            LB.setPower(-0.4);
            RF.setPower(-0.4);
            LF.setPower(-0.4);
        }
        if (gamepad1.right_trigger>0) {
            RB.setPower(0.4);
            LB.setPower(0.4);
            RF.setPower(0.4);
            LF.setPower(0.4);
        }

        //arm motor controls
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            Arm.setPower(0.08);
        }
        if (gamepad2.right_trigger == 1) {
            Arm.setPower(0.5);
        }
        if (gamepad2.left_trigger == 1) {
            Arm.setPower(-0.5);
        }
        if (gamepad2.y) {
            Arm.setPower(-0.13);
        }

        //claw controls

        if (gamepad2.a) {
            leftServo.setPosition(0.3);
            rightServo.setPosition(0.3);
        }

        if (gamepad2.b) {
            leftServo.setPosition(-0.2);
            rightServo.setPosition(0.8);
        }

        if (gamepad2.right_bumper) {
            angleServo.setPosition(-0.8);
            //servoAngle = 0.1;
        }
        if (gamepad2.left_bumper) {
            angleServo.setPosition(0.98);
            //servoAngle =0.00;
        }
        if (gamepad2.x) {
            angleServo.setPosition(-1.00);
        }
        if (gamepad2.x && gamepad2.dpad_right && gamepad2.dpad_left) {
            planeServo.setPosition(0.4);
            //servoAngle =0.00;
        }

        //angleServo.setPosition(servoAngle);




    }






}