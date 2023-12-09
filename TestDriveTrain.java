package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestDriveTrain")
public class TestDriveTrain extends OpMode {

    DcMotor RB, LB, RF, LF, Arm, Actuator;
    Servo angleServo1, angleServo2, rightServo, leftServo, planeServo;
    public static double MOTOR_MULTIPLIER = 0.75; //default
    private static final double servoOpen = 0.3;


    @Override
    public void init() {

        RB = hardwareMap.dcMotor.get("Left_Back_Motor");
        LB = hardwareMap.dcMotor.get("Right_Back_Motor");
        RF = hardwareMap.dcMotor.get("Left_Front_Motor");
        LF = hardwareMap.dcMotor.get("Right_Front_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        planeServo = hardwareMap.get(Servo.class, "servo4");
        Actuator = hardwareMap.dcMotor.get("Actuator_Motor");

        planeServo.setPosition(0);
        Arm.setPower(0.23);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        initArm();


    }

    public void motion() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x*0.7;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        LF.setPower(frontLeftPower*MOTOR_MULTIPLIER);
        LB.setPower(backLeftPower*MOTOR_MULTIPLIER);
        RF.setPower(frontRightPower*MOTOR_MULTIPLIER);
        RB.setPower(backRightPower*MOTOR_MULTIPLIER);
    }

    public void speedChange() {
        if (gamepad1.dpad_up) {
            MOTOR_MULTIPLIER = 1.0;
        } else if (gamepad1.dpad_right) {
            MOTOR_MULTIPLIER = 0.75;
        } else if (gamepad1.dpad_down) {
            MOTOR_MULTIPLIER = 0.5;
        } else if (gamepad1.dpad_left) {
            MOTOR_MULTIPLIER = 0.25;
        }
    }

    public void initArm() {
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void pickUpPixel() {
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(-0.4);

        while (Arm.isBusy()) {
            System.out.println("a");
        }

        // Stop the motor after reaching the target position
        Arm.setPower(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftServo.setPosition(-1);
        rightServo.setPosition(0.75);

        Arm.setTargetPosition(10);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.4);

        while (Arm.isBusy()) {
            System.out.println("a");
        }

        // Stop the motor after reaching the target position
        Arm.setPower(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void moveArmBack()  {
        Arm.setTargetPosition(100);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.4);

        while (Arm.isBusy()) {
            angleServo1.setPosition(1.00);
            angleServo2.setPosition(-0.99);
        }

        // Stop the motor after reaching the target position
        Arm.setPower(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void releaseFirstPixel() {
        rightServo.setPosition(servoOpen);
    }

    public void releaseSecondPixel() {
        leftServo.setPosition(servoOpen);

        Arm.setTargetPosition(10);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(-0.4);

        while (Arm.isBusy()) {
            System.out.println("a");
        }

        // Stop the motor after reaching the target position
        Arm.setPower(0);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {

        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        motion();

        speedChange();

        if (gamepad2.a) {
            pickUpPixel();
        }

        if (gamepad2.b) {
            moveArmBack();
        }

        if (gamepad2.right_trigger > 0.01) {
            releaseFirstPixel();
        }

        if (gamepad2.left_trigger > 0.01) {
            releaseSecondPixel();
        }

        if (gamepad2.right_bumper && gamepad2.left_bumper) {
            planeServo.setPosition(0.4);
            //servoAngle =0.00;
        }

        if (gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1) {
            Actuator.setPower(-gamepad2.left_stick_y);
        }
        else {
            Actuator.setPower(0);
        }
    }
}