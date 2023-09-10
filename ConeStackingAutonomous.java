package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Cone Stacking Autonomous")
public class ConeStackingAutonomous extends LinearOpMode {

    // Declare motors and servos
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    private Servo servoGripper;
    private Servo servoArm;

    // Set constants for motor and servo movements
    private static final double GRIPPER_OPEN_POSITION = 0.5;
    private static final double GRIPPER_CLOSED_POSITION = 0.1;
    private static final double ARM_UP_POSITION = 0.7;
    private static final double ARM_DOWN_POSITION = 0.3;
    private static final double MOTOR_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servos
        leftBackMotor = hardwareMap.dcMotor.get("left_motor");
        rightBackMotor = hardwareMap.dcMotor.get("right_motor");
        leftFrontMotor = hardwareMap.dcMotor.get("left_motor");
        rightFrontMotor = hardwareMap.dcMotor.get("right_motor");
        servoGripper = hardwareMap.servo.get("gripper_servo");
        servoArm = hardwareMap.servo.get("arm_servo");

        // Wait for start button to be pressed
        waitForStart();

        // Move forward to position robot under cones
        leftBackMotor.setPower(MOTOR_POWER);
        rightBackMotor.setPower(MOTOR_POWER);
        leftFrontMotor.setPower(MOTOR_POWER);
        rightFrontMotor.setPower(MOTOR_POWER);
        Thread.sleep(1000);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

        // Lower arm to pick up cone
        servoArm.setPosition(ARM_DOWN_POSITION);
        Thread.sleep(500);

        // Close gripper to grab cone
        servoGripper.setPosition(GRIPPER_CLOSED_POSITION);
        Thread.sleep(500);

        // Raise arm to lift cone
        servoArm.setPosition(ARM_UP_POSITION);
        Thread.sleep(500);

        // Move backward to position cone over stick
        leftFrontMotor.setPower(-MOTOR_POWER);
        leftBackMotor.setPower(-MOTOR_POWER);
        rightFrontMotor.setPower(-MOTOR_POWER);
        rightBackMotor.setPower(-MOTOR_POWER);
        Thread.sleep(1000);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        // Lower arm to place cone on stick
        servoArm.setPosition(ARM_DOWN_POSITION);
        Thread.sleep(500);

        // Open gripper to release cone
        servoGripper.setPosition(GRIPPER_OPEN_POSITION);
        Thread.sleep(500);

        // Raise arm to complete stacking action
        servoArm.setPosition(ARM_UP_POSITION);
    }
}