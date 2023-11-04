/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Red Put Pixel")
//@Disabled
public class RedConceptTensorFlowObjectDetection extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231027_204348.tflite";
    private static final String[] LABELS = {
            "red_object",
    };

    DcMotor leftF;
    DcMotor leftB;
    DcMotor rightF;
    DcMotor rightB;

    DcMotor Arm;

    //DistanceSensor distanceSensor;
    Servo angleServo;
    Servo rightServo;
    Servo leftServo;
/*

    DcMotor arm = hardwareMap.get(DcMotor.class, "Arm_Motor");
    Servo l = hardwareMap.get(Servo.class, "servo3");
    Servo r = hardwareMap.get(Servo.class, "servo2");*/

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

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
        leftServo.setPosition(-0.8);
        rightServo.setPosition(0.8);
        Arm.setPower(0.08);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        telemetry.update();

        waitForStart();

        //if (opModeIsActive()) {
        while (opModeIsActive()) {
            //INTIALIZE SERVOS TO BE ON SO THE PIXEL DOESN'T COME OFF
            Arm.setPower(0.28);
            sleep(500);
            Arm.setPower(0.08);
            leftF.setPower(-0.1);
            leftB.setPower(-0.1);
            rightB.setPower(0.1);
            rightF.setPower(0.1);
            sleep(2000);
            leftF.setPower(0);
            leftB.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            sleep(500);

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Recs", currentRecognitions);
            telemetry.update();
//5700
            if(currentRecognitions.size() != 0){
                leftF.setPower(-0.1);
                leftB.setPower(-0.1);
                rightB.setPower(0.1);
                rightF.setPower(0.1);
                sleep(4500);
                leftF.setPower(0);
                leftB.setPower(0);
                rightB.setPower(0);
                rightF.setPower(0);
                sleep(1000);
                /* Prelim Rotation Code Adjust as Nessesary
                leftF.setPower(-0.1);
                leftB.setPower(-0.1);
                rightB.setPower(0.1);
                rightF.setPower(0.1);
                sleep(VALUE);
                */
                angleServo.setPosition(0.98);
                sleep(3000);
                leftF.setPower(-0.1);
                leftB.setPower(-0.1);
                rightB.setPower(0.1);
                rightF.setPower(0.1);
                sleep(1300);
                leftServo.setPosition(0.3);
                rightServo.setPosition(0.3);
                sleep(500);
                break;
            }
            else{
                //FIX THIS ROTATION, THEN HAVE THE ROBOT DRIVE FORWARD AND PUT THE PIXEL DOWN
                leftB.setPower(-0.2);
                leftF.setPower(-0.2);
                rightF.setPower(-0.2);
                rightB.setPower(-0.2);
                sleep(1250);

                leftB.setPower(0);
                leftF.setPower(0);
                rightF.setPower(0);
                rightB.setPower(0);
                sleep(500);

                currentRecognitions = tfod.getRecognitions();
                telemetry.addData("Recs", currentRecognitions);
                telemetry.update();
                if (currentRecognitions.size()!= 0){
                    leftF.setPower(-0.1);
                    leftB.setPower(-0.1);
                    rightB.setPower(0.1);
                    rightF.setPower(0.1);
                    sleep(2000);
                    leftF.setPower(0);
                    leftB.setPower(0);
                    rightB.setPower(0);
                    rightF.setPower(0);
                    angleServo.setPosition(0.98);
                    sleep(2500);
                    leftF.setPower(-0.1);
                    leftB.setPower(-0.1);
                    rightB.setPower(0.1);
                    rightF.setPower(0.1);
                    sleep(2400);
                    leftF.setPower(0);
                    leftB.setPower(0);
                    rightB.setPower(0);
                    rightF.setPower(0);
                    leftServo.setPosition(0.3);
                    rightServo.setPosition(0.3);
                    sleep(500);

                    break;
                }
                else {
                    leftB.setPower(0.2);
                    leftF.setPower(0.2);
                    rightF.setPower(0.2);
                    rightB.setPower(0.2);
                    sleep(1900);

                    leftF.setPower(-0.1);
                    leftB.setPower(-0.1);
                    rightB.setPower(0.1);
                    rightF.setPower(0.1);
                    sleep(2000);

                    leftF.setPower(0);
                    leftB.setPower(0);
                    rightB.setPower(0);
                    rightF.setPower(0);
                    angleServo.setPosition(0.98);
                    sleep(2500);
                    leftF.setPower(-0.1);
                    leftB.setPower(-0.1);
                    rightB.setPower(0.1);
                    rightF.setPower(0.1);
                    sleep(2000);

                    leftF.setPower(0);
                    leftB.setPower(0);
                    rightB.setPower(0);
                    rightF.setPower(0);
                    leftServo.setPosition(0.3);
                    rightServo.setPosition(0.3);
                    sleep(500);
                    break;
                }

            }

            // Push telemetry to the Driver Station.

                /*// Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);*/
        }
    }

    // Save more CPU resources when camera is no longer needed.
    //visionPortal.close();

    // }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.65f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private List<Recognition> telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return currentRecognitions;
    }   // end method telemetryTfod()

}   // end class
