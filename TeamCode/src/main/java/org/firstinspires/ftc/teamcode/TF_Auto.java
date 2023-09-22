package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.tfod.TfodParameters.CurrentGame.LABELS;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous
public class TF_Auto {
    AprilTagProcessor myAprilTagProcessor;
    public void initAprilTag() {
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
    }
    public void initTFOD(){
        TfodProcessor.Builder myTfodProcessorBuilder;
        TfodProcessor myTfodProcessor;

// Create a new TFOD Processor Builder object.
        myTfodProcessorBuilder = new TfodProcessor.Builder();

// Optional: set other custom features of the TFOD Processor (4 are shown here).
        myTfodProcessorBuilder.setMaxNumRecognitions(10);  // Max. number of recognitions the network will return
        myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
        myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
        myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track

// Create a TFOD Processor by calling build()
        myTfodProcessor = myTfodProcessorBuilder.build();
    }

}
