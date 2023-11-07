package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoMove extends OpMode {
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
    }

    @Override
    public void loop() {

    }
    public void dropPixel(){
        Arm.setPower(-0.4);
        angleServo.setPosition(1);
        leftServo.setPosition(1);
        rightServo.setPosition(1);
    }
    public void rotate(){
        RB.setPower(0.3);
        RF.setPower(-0.3);
        LB.setPower(0.3);
        LF.setPower(-0.3);
    }
    public void moveForward(){
        RB.setPower(0.4);
        RF.setPower(0.4);
        LB.setPower(0.4);
        LF.setPower(0.4);
    }
    public void moveBackward(){
        RB.setPower(0.4);
        RF.setPower(0.4);
        LB.setPower(0.4);
        LF.setPower(0.4);
    }
    public void stopMoving(){
        RB.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        LF.setPower(0);
    }


}
