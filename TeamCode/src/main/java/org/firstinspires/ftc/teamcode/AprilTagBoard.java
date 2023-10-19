package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class AprilTagBoard extends LinearOpMode {
    AprilTagProcessor myAprilTagProcessor;
// Create the AprilTag processor and assign it to a variable.
    myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
    TfodProcessor myTfodProcessor;
// Create the TensorFlow Object Detection processor and assign it to a variable.
    myTfodProcessor = TfodProcessor.easyCreateWithDefaults();
    VisionPortal myVisionPortal;

// Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
    myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
    myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
