package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="auto", group="autonomous")
//@Disabled
public class autotesting extends LinearOpMode {

    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        //sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor");
        leftBackMotor = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hardwareMap.dcMotor.get("Right_Back_Motor");
        leftFrontMotor = hardwareMap.dcMotor.get("Left_Front_Motor");
        rightFrontMotor = hardwareMap.dcMotor.get("Right_Front_Motor");


        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            leftBackMotor.setPower(0.5);
            rightBackMotor.setPower(-0.5);
            leftFrontMotor.setPower(0.5);
            rightFrontMotor.setPower(0.5);
            sleep(500);

        }

    }
}
