package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SleeveDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Signal Sleeve Test")
public class VisionTest extends LinearOpMode {
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        leftBackMotor = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightBackMotor = hardwareMap.dcMotor.get("Right_Back_Motor");
        leftFrontMotor = hardwareMap.dcMotor.get("Left_Front_Motor");
        rightFrontMotor = hardwareMap.dcMotor.get("Right_Front_Motor");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }
        waitForStart();
        if(opModeIsActive()==true){
            for (int i = 0; i < 1; i++) {
                    if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER) {
                        leftBackMotor.setPower(0.5);
                        rightBackMotor.setPower(-0.5);
                        leftFrontMotor.setPower(0.5);
                        rightFrontMotor.setPower(0.5);
                        sleep(635);

                        leftBackMotor.setPower(0);
                        rightBackMotor.setPower(0);
                        leftFrontMotor.setPower(0);
                        rightFrontMotor.setPower(0);

                    }
                    if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT) {
                        leftBackMotor.setPower(0.5);
                        rightBackMotor.setPower(-0.5);
                        leftFrontMotor.setPower(0.5);
                        rightFrontMotor.setPower(0.5);
                        sleep(660);

                        leftBackMotor.setPower(0);
                        rightBackMotor.setPower(0);
                        leftFrontMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        sleep(800);

                        rightBackMotor.setPower(0.5);
                        rightFrontMotor.setPower(0.5);
                        leftBackMotor.setPower(0.5);
                        leftFrontMotor.setPower(-0.5);
                        sleep(925);

                        leftBackMotor.setPower(0);
                        rightBackMotor.setPower(0);
                        leftFrontMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        sleep(3000000);
                    }
                    if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT) {
                        leftBackMotor.setPower(0.5);
                        rightBackMotor.setPower(-0.5);
                        leftFrontMotor.setPower(0.5);
                        rightFrontMotor.setPower(0.5);
                        sleep(700);

                        leftBackMotor.setPower(0);
                        rightBackMotor.setPower(0);
                        leftFrontMotor.setPower(0);
                        rightFrontMotor.setPower(0);
                        sleep(800);

                        rightBackMotor.setPower(-0.5);
                        rightFrontMotor.setPower(-0.5);
                        leftBackMotor.setPower(-0.5);
                        leftFrontMotor.setPower(0.5);
                        sleep(925);

                        leftBackMotor.setPower(0);
                        rightBackMotor.setPower(0);
                        leftFrontMotor.setPower(0);
                        rightFrontMotor.setPower(0);

                    }


            }

    }
    }}