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

    int servoAngle;






    @Override
    public void init() {

        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");
        Arm = hardwareMap.dcMotor.get("Arm");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        angleServo = hardwareMap.get(Servo.class, "servo1");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");

        servoAngle = 0;

        angleServo.setPosition(servoAngle);
        rightServo.setPosition(0);
        leftServo.setPosition(0);


    }


    public void loop() {

        //rotations
        LB.setPower(-gamepad1.left_stick_x);
        LF.setPower(-gamepad1.left_stick_x);
        RB.setPower(-gamepad1.left_stick_x);
        RF.setPower(-gamepad1.left_stick_x);

        //left and right
        if (gamepad1.right_stick_x < 0) {
            LB.setPower(-(gamepad1.right_stick_x));
            LF.setPower((gamepad1.right_stick_x));
            RB.setPower((gamepad1.right_stick_x));
            RF.setPower(-(gamepad1.right_stick_x));
        }

        if (gamepad1.right_stick_x > 0) {
            LB.setPower(-(gamepad1.right_stick_x));
            LF.setPower((gamepad1.right_stick_x));
            RB.setPower((gamepad1.right_stick_x));
            RF.setPower(-(gamepad1.right_stick_x));
        }


        //forward + backward controls - left stick up and down | placed below rotations because it has priority
        LB.setPower(gamepad1.right_stick_y);
        LF.setPower(gamepad1.right_stick_y);
        RB.setPower(-gamepad1.right_stick_y);
        RF.setPower(-gamepad1.right_stick_y);

        Arm.setPower(gamepad1.right_trigger/2.5);
        Arm.setPower(-gamepad1.left_trigger/2.5);

        if (gamepad1.a) {
            leftServo.setPosition(0.3);
            rightServo.setPosition(0.3);
        }

        if (gamepad1.b) {
            leftServo.setPosition(0);
            rightServo.setPosition(0);
        }

        if (gamepad1.right_bumper && servoAngle < 1) {
            servoAngle += 0.1;
        } else if (gamepad1.left_bumper && servoAngle > 0) {
            servoAngle -=0.1;
        }

        angleServo.setPosition(servoAngle);




    }






}