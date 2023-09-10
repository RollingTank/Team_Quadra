package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Gamepad", group="Linear Opmode")
//@Disabled
public class Gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor slideMotor;
    Servo rightGear;
    Servo leftGear;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftBackMotor = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hardwareMap.dcMotor.get("Right_Back_Motor");
//        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor = hardwareMap.dcMotor.get("Left_Front_Motor");
        rightFrontMotor = hardwareMap.dcMotor.get("Right_Front_Motor");
        slideMotor = hardwareMap.dcMotor.get("Linear_Slide_Motor");
        rightGear = hardwareMap.servo.get("Right_Gear");
        leftGear = hardwareMap.servo.get("Left_Gear");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            if (!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down) {
                leftBackMotor.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);
                rightBackMotor.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
                leftFrontMotor.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);
                rightFrontMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            }

            if (gamepad1.left_bumper == true) {
                slideMotor.setPower(0.5);
            }
            if (gamepad1.right_bumper == true) {
                slideMotor.setPower(-0.6);
            }
            if (gamepad1.left_bumper == false && gamepad1.right_bumper == false) {
                slideMotor.setPower(-0.1);
            }
            idle();

/*            if (gamepad1.left_stick_x > 0 && gamepad1.right_stick_x > 0) {
                rightBackMotor.setPower(-0.5);
                rightFrontMotor.setPower(-0.5);
                leftBackMotor.setPower(-0.5);
                leftFrontMotor.setPower(0.5);
            }
            if (gamepad1.left_stick_x < 0 && gamepad1.right_stick_x < 0) {
                rightBackMotor.setPower(0.5);
                rightFrontMotor.setPower(0.5);
                leftBackMotor.setPower(0.5);
                leftFrontMotor.setPower(-0.5);
            }
            if (gamepad1.left_stick_y < 0 && gamepad1.right_stick_y < 0){
                leftBackMotor.setPower(-0.5);
                rightBackMotor.setPower(0.5);
                leftFrontMotor.setPower(-0.5);
                rightFrontMotor.setPower(-0.5);
            }
            if (gamepad1.left_stick_y > 0 && gamepad1.right_stick_y > 0){
                leftBackMotor.setPower(0.5);
                rightBackMotor.setPower(-0.5);
                leftFrontMotor.setPower(0.5);
                rightFrontMotor.setPower(0.5);
            }*/

            if (gamepad1.dpad_down == true) {
                leftBackMotor.setPower(0.3);
                rightBackMotor.setPower(-0.3);
                leftFrontMotor.setPower(0.3);
                rightFrontMotor.setPower(0.3);
            }
            else if (gamepad1.dpad_up == true) {
                leftBackMotor.setPower(-0.3);
                rightBackMotor.setPower(0.3);
                leftFrontMotor.setPower(-0.3);
                rightFrontMotor.setPower(-0.3);
            }
            else if (gamepad1.dpad_right == true) {
                rightBackMotor.setPower(0.3);
                rightFrontMotor.setPower(0.3);
                leftBackMotor.setPower(0.3);
                leftFrontMotor.setPower(-0.3);
            }
            else if (gamepad1.dpad_left == true) {
                rightBackMotor.setPower(-0.3);
                rightFrontMotor.setPower(-0.3);
                leftBackMotor.setPower(-0.3);
                leftFrontMotor.setPower(0.3);
            }

            if (gamepad1.a == true) {
                // move to 0 degrees.
                leftGear.setPosition(0.32);
                rightGear.setPosition(0.32);
            }
            if (gamepad1.b == true) {
                // move to 90 degrees.
                leftGear.setPosition(-0.3);
                rightGear.setPosition(0.67);
            }

            idle();
        }
    }
}