package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Autonomous(name = "NewAutoTestCode")
public class BlueAuto extends LinearOpMode {

    DcMotor leftF;
    DcMotor leftB;
    DcMotor rightF;
    DcMotor rightB;
    DcMotor Arm;
    Servo angleServo;
    Servo rightServo;
    Servo leftServo;

    private static final double lsClosePosition = -0.8; //left servo close angle
    private static final double rsClosePosition = 0.8; //right servo close angle
    private static final double asDownPosition = 0.98; //as = angle servo down angle
    public static final int inmsConversion = 42; //conversion rate for centimeters to milliseconds | forward/backward movement
    public static final int inmsConversionStrafe = 10; //conversion rate for centimeters to milliseconds | left/right movement
    public static final int dgmsConversion = 10; //conversion rate for degrees to milliseconds
    public static final double powerMovementConstant = 0.5; //power value when robot is set to move forward or backward
    public static final double powerStrafeConstant = 0.8; //power value when robot is set to strafe left or right
    public static double powerRotateConstant = 0.2; //power value when robot is set to rotate

    public int inToMs(int inches) {
        return inches * inmsConversion; //converts centimeters to milliseconds
    }

    public int inToMsStrafe(int inches) {
        return inches * inmsConversionStrafe;
    }

    public int dgToMs(int degrees) {
        return degrees * dgmsConversion; //converts degrees to milliseconds
    }

    public void angleServoDown() {
        angleServo.setPosition(asDownPosition); //function to set angle servo down
        sleep(500);
    }

    public void moveForward(int inches) {
        leftF.setPower(-powerMovementConstant); //moves robot forward based on input (centimeters)
        leftB.setPower(-powerMovementConstant);
        rightB.setPower(powerMovementConstant);
        rightF.setPower(powerMovementConstant);
        sleep(inToMs(inches));
    }

    public void moveBackward(int inches) {
        leftF.setPower(powerMovementConstant); //moves robot backwards based on input (centimeters)
        leftB.setPower(powerMovementConstant);
        rightB.setPower(-powerMovementConstant);
        rightF.setPower(-powerMovementConstant);
        sleep(inToMs(inches));
    }
    public void strafeRight(int inches) {
        leftF.setPower(-powerStrafeConstant); //moves robot right based on input (centimeters)
        leftB.setPower(powerStrafeConstant);
        rightB.setPower(powerStrafeConstant);
        rightF.setPower(-powerStrafeConstant);
        sleep(inToMsStrafe(inches));
    }

    public void strafeLeft(int inches) {
        leftF.setPower(powerStrafeConstant); //moves robot left based on input (centimeters)
        leftB.setPower(-powerStrafeConstant);
        rightB.setPower(-powerStrafeConstant);
        rightF.setPower(powerStrafeConstant);
        sleep(inToMsStrafe(inches));
    }

    public void stopMovement(int milliseconds) {
        leftF.setPower(0); //stops robot for a set amount of time (milliseconds)
        leftB.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        sleep(milliseconds);
    }

    public void rotate(int degrees) {

        if (degrees < 0) {
            powerRotateConstant*=-1;
        }

        leftF.setPower(powerRotateConstant); //rotates robot based on input (degrees) | positive degrees move robot counter-clockwise while negative degrees will move robot clockwise
        leftB.setPower(powerRotateConstant);
        rightB.setPower(powerRotateConstant);
        rightF.setPower(powerRotateConstant);
        sleep(dgToMs(Math.abs(degrees)));

        if (degrees < 0) {
            powerRotateConstant*=-1;
        }
    }

    public void parkSequenceLeft() {

    }

    public void parkSequenceMiddle() {

    }

    public void parkSequenceRight() {

    }

    @Override
    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("Left_Front_Motor");
        leftB = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightF = hardwareMap.dcMotor.get("Right_Front_Motor");
        rightB = hardwareMap.dcMotor.get("Right_Back_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo = hardwareMap.get(Servo.class, "servo1");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        leftServo.setPosition(lsClosePosition);
        rightServo.setPosition(rsClosePosition);
        Arm.setPower(0.09);

        waitForStart();

        //while (opModeIsActive()) {
            /*leftF.setPower(-0.5); //moves robot forward based on input (centimeters)
            leftB.setPower(-0.5);
            rightB.setPower(0.5);
            rightF.setPower(0.5);
            sleep(1000);
            leftF.setPower(0); //moves robot forward based on input (centimeters)
            leftB.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            sleep(1000);
            */
        leftF.setPower(-powerStrafeConstant); //moves robot right based on input (centimeters)
        leftB.setPower(powerStrafeConstant);
        rightB.setPower(powerStrafeConstant);
        rightF.setPower(-powerStrafeConstant);
        sleep(2000);
        //}

    }

}


