package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "NewAutoTestCode")
public class BlueAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_20231018_181921.tflite";
    private static final String[] LABELS = {
            "team object",
    };
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    DcMotor leftF;
    DcMotor leftB;
    DcMotor rightF;
    DcMotor rightB;
    DcMotor Arm;
    Servo angleServo1;
    Servo angleServo2;
    Servo rightServo;
    Servo leftServo;

    private static final double rsOpenPosition = 0.3;
    private static final double lsOpenPosition = 0.3;
    private static final double lsClosePosition = -0.8; //left servo close angle
    private static final double rsClosePosition = 0.8; //right servo close angle
    public static final int inmsConversion = 40; //conversion rate for centimeters to milliseconds | forward/backward movement
    public static final int inmsConversionStrafe = 35; //conversion rate for centimeters to milliseconds | left/right movement
    public static final int dgmsConversion = 40; //conversion rate for degrees to milliseconds
    public static final double powerMovementConstant = 0.5; //power value when robot is set to move forward or backward
    public static final double powerStrafeConstant = 0.8; //power value when robot is set to strafe left or right
    public static double powerRotateConstant = 0.2; //power value when robot is set to rotate

    public int inToMs(int inches) { return inches * inmsConversion; } //converts centimeters to milliseconds for forward and backward movements

    public int inToMsStrafe(int inches) { return inches * inmsConversionStrafe; } //converts centimeters to milliseconds for side to side movements

    public int dgToMs(int degrees) { return degrees * dgmsConversion; } //converts degrees to milliseconds

    public void angleServoDown() {
        angleServo1.setPosition(0.965); //sets angle servo down to place pixel on floor
        angleServo2.setPosition(-1.00);
        sleep(2000);
    }

    public void angleServoUp() {
        angleServo1.setPosition(-1.00); //sets angle servo up to place pixel on board
        angleServo2.setPosition(1.00);
        sleep(2000);
    }

    public void releaseFirstPixel() {
        rightServo.setPosition(rsOpenPosition);
        sleep(1000);
    }

    public void releaseSecondPixel() {
        leftServo.setPosition(lsOpenPosition);
        sleep(1000);
    }

    public void stopMovement(int milliseconds) {
        leftF.setPower(0); //stops robot for a set amount of time (milliseconds)
        leftB.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        sleep(milliseconds);
    }

    public void moveForward(int inches) {
        leftF.setPower(-powerMovementConstant); //moves robot forward based on input (centimeters)
        leftB.setPower(-powerMovementConstant);
        rightB.setPower(powerMovementConstant);
        rightF.setPower(powerMovementConstant);
        sleep(inToMs(inches));
        stopMovement(1);
    }

    public void moveBackward(int inches) {
        leftF.setPower(powerMovementConstant); //moves robot backwards based on input (centimeters)
        leftB.setPower(powerMovementConstant);
        rightB.setPower(-powerMovementConstant);
        rightF.setPower(-powerMovementConstant);
        sleep(inToMs(inches));
        stopMovement(1);
    }
    public void strafeRight(int inches) {
        leftF.setPower(-powerStrafeConstant); //moves robot right based on input (centimeters)
        leftB.setPower(powerStrafeConstant);
        rightB.setPower(powerStrafeConstant);
        rightF.setPower(-powerStrafeConstant);
        sleep(inToMsStrafe(inches));
        stopMovement(1);
    }

    public void strafeLeft(int inches) {
        leftF.setPower(powerStrafeConstant); //moves robot left based on input (centimeters)
        leftB.setPower(-powerStrafeConstant);
        rightB.setPower(-powerStrafeConstant);
        rightF.setPower(powerStrafeConstant);
        sleep(inToMsStrafe(inches));
        stopMovement(1);
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

        stopMovement(1);
    }
    @Override
    public void runOpMode() {
        leftF = hardwareMap.dcMotor.get("Left_Front_Motor");
        leftB = hardwareMap.dcMotor.get("Left_Back_Motor");
        rightF = hardwareMap.dcMotor.get("Right_Front_Motor");
        rightB = hardwareMap.dcMotor.get("Right_Back_Motor");
        Arm = hardwareMap.dcMotor.get("Arm_Motor");
        angleServo1 = hardwareMap.get(Servo.class, "servo1");
        angleServo2 = hardwareMap.get(Servo.class, "servo5");
        rightServo = hardwareMap.get(Servo.class, "servo2");
        leftServo = hardwareMap.get(Servo.class, "servo3");
        leftServo.setPosition(lsClosePosition);
        rightServo.setPosition(rsClosePosition);
        Arm.setPower(0.09);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        telemetry.update();
        waitForStart();

        //add start move forward code here

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Recs", currentRecognitions);
        telemetry.update();

        moveForward(12);

        angleServoDown();

        releaseFirstPixel();

        angleServoUp();

        Arm.setPower(0.8);

        releaseSecondPixel();

        Arm.setPower(0);





    }

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

    }

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
    }

}

/* if (currentRecognitions.size() != 0) {
            moveForward(12);
            moveBackward(8); //move forward then backward to push the object out of the way (just incase)
            angleServoDown();
            releaseFirstPixel();

            Arm.setPower(0.89); //lift arm up
            rotate(90);
            moveBackward(12);
            angleServoUp();
            releaseSecondPixel();
            Arm.setPower(0);

            strafeRight(8);
            moveBackward(10);

        } else {
            moveForward(2);
            rotate(90); //slightly move forward, then rotate 90 degrees to the right, then move backwards to get a clear view
            moveBackward(8);

            currentRecognitions = tfod.getRecognitions();
            telemetry.addData("Recs", currentRecognitions);
            telemetry.update();
            sleep(500);

            if (currentRecognitions.size() != 0) {
                moveForward(4); //move forward, then place the pixel
                angleServoDown();
                releaseFirstPixel();


            } else {

                moveForward(6); //move all the way forward, turn around, place the pixel
                rotate(180);
                angleServoDown();
                releaseFirstPixel();

            }


        }


 */
