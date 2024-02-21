package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRed;
import org.firstinspires.ftc.teamcode.common.vision.openREDClose;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRed;

import java.util.List;

//camera imports
import org.firstinspires.ftc.teamcode.common.vision.openREDFar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import com.acmerobotics.dashboard.config.Config;
//import org.checkerframework.checker.signedness.qual.Constant;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous
public class RedClosePreload extends LinearOpMode {
    RobotHardware robot;
    OpModeVisionRed pipeline;
    OpenCvWebcam camera;

    public enum Location{
        Left,
        Right,
        Middle
    };


    @Override
    public void runOpMode() {
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewID);
        pipeline = new OpModeVisionRed();
        camera.setPipeline(pipeline);

        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("No Camera Could Not Start");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

       // pipeline = new openREDClose();

        robot = new RobotHardware(hardwareMap);

        Globals.IS_AUTO = true;
        Globals.IS_BLUE = false;
        Globals.IS_CLOSE = true;

        robot.init();



        TrajectorySequence right = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringPositions.YELLOW_PIXEL_POSITIONS[5])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(43.4, -37.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, -57.5, Math.toRadians(181))).back(6).build();

        TrajectorySequence center = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringPositions.YELLOW_PIXEL_POSITIONS[4])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.5).lineToLinearHeading(new Pose2d(37, -25.3, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, -57.5, Math.toRadians(181))).back(6).build();

        TrajectorySequence left = robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.yellowPixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.yellowPixelExtendCommand();
                }).lineToLinearHeading(ScoringPositions.YELLOW_PIXEL_POSITIONS[3])
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.RIGHT);
                }).UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.slideRetractCommand();
                }).UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    robot.limitArm(0.6);
                    robot.purplePixelArmCommand();
                }).UNSTABLE_addTemporalMarkerOffset(1.4, () -> {
                    robot.limitArm(0.35);
                }).waitSeconds(1.8).lineToLinearHeading(new Pose2d(21.5, -39.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.limitArm(0.55);
                    robot.claw.setClawState(Claw.Mode.SHARP, Claw.Mode.LEFT);
                }).waitSeconds(0.7).UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.resetCommand();
                }).lineToLinearHeading(new Pose2d(48, -57.5, Math.toRadians(181))).back(6).build();

        while (opModeInInit()) {
            robot.update();
        }


        waitForStart();
        if (isStopRequested()) return;

      //  OpModeVisionRed cameraVision = new OpModeVisionRed();
        OpModeVisionRed.Location objLocation = pipeline.location;
        int numberOfLeft = 0;
        int numberOfRight = 0;
        int numberOfCenter = 0;

        for (int i=0; i<100; i++) {
            if (objLocation == OpModeVisionRed.Location.Middle) {
                numberOfCenter += 1;
                telemetry.addData("block detected in center", objLocation);
                telemetry.update();
            }
            else if (objLocation == OpModeVisionRed.Location.Right) {
            numberOfRight += 1;
            telemetry.addData("block detected in right", objLocation);
            telemetry.update();
        }
        else if (objLocation == OpModeVisionRed.Location.Left) {
            numberOfLeft += 1;
            telemetry.addData("block detected in left", objLocation);
            telemetry.update();
        }
        else {
            telemetry.addData("no block detected", objLocation);
            telemetry.update();
        }}



        if (numberOfLeft>numberOfCenter && numberOfLeft>numberOfRight) {
            //robot.drive.followTrajectorySequenceAsync(left);
            telemetry.addData("following left trajectory", objLocation);
            telemetry.update();
        }
        else if (numberOfRight>=numberOfLeft && numberOfRight>=numberOfCenter) {
            //robot.drive.followTrajectorySequenceAsync(right);
            telemetry.addData("following right trajectory", objLocation);
            telemetry.update();
        }
        else if (numberOfCenter>=numberOfLeft && numberOfCenter>=numberOfRight) {
            //robot.drive.followTrajectorySequenceAsync(left);
            telemetry.addData("following center trajectory", objLocation);
            telemetry.update();
        }
        else {
            telemetry.addData("no trajectory being followed", objLocation);
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            robot.update();
        }

        PoseStorage.currentPose = robot.drive.getPoseEstimate();
    }
}