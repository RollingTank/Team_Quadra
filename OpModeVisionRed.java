package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRed;
import org.firstinspires.ftc.teamcode.common.vision.openREDClose;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.PoseStorage;
import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionRed;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.centerstage.Globals;
import org.firstinspires.ftc.teamcode.common.centerstage.ScoringPositions;
import org.firstinspires.ftc.teamcode.common.hardware.Claw;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.teamcode.common.vision.openREDClose;
import org.firstinspires.ftc.teamcode.common.vision.openBLUEFar;
import org.firstinspires.ftc.teamcode.common.vision.openBLUEFar;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpModeVisionRed extends OpenCvPipeline {

    public double leftValue;
    public double rightValue;
    public double middleValue;
    public static boolean DETECT_RED = false;
    public int locationFinal;
    private Paint linePaint;
    public static double MINIMUM_VALUES = 40;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_LOW_HUE = 50;
    public static double MAXIMUM_BLUE_LOW_HUE = 150;
    public static double MINIMUM_BLUE_HIGH_HUE = 100;
    public static double MAXIMUM_BLUE_HIGH_HUE = 160;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;

    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location{
        Left,
        Right,
        Middle,
        Unfound
    };
    public OpModeVisionRed.Location location;

    static final Rect Left_ROI = new Rect(new Point(0,400), new Point(250,150));
    static final Rect Middle_ROI = new Rect(new Point(400,400), new Point(600,160));
    static final Rect Right_ROI = new Rect(new Point(700, 400), new Point(950,150));
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public void CenterStageCVDetection(Telemetry t){
        telemetry = t;
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);

        if(!DETECT_RED) {
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat);
        }
        else {
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1,mat2,mat);
        }

        Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);
        Mat middle = mat.submat(Middle_ROI);

        leftValue = Core.sumElems(left).val[0];
        rightValue = Core.sumElems(right).val[0];
        middleValue = Core.sumElems(middle).val[0];

        //telemetry.addData("left raw val:", leftValue);
        //telemetry.addData("right raw val:", rightValue);
        //telemetry.addData("middle raw val:", middleValue);

        left.release();
        right.release();
        middle.release();

        if (leftValue > rightValue && leftValue > middleValue) {
            location = OpModeVisionRed.Location.Left;
            //telemetry.addData("location:", "left");
        } else if (rightValue > middleValue && rightValue > leftValue) {
            location = OpModeVisionRed.Location.Right;
            //telemetry.addData("location:", "right");
        } else if(middleValue > rightValue && middleValue > leftValue){
            location = OpModeVisionRed.Location.Middle;
            //telemetry.addData("location:", "middle");
        }
        else {
            location = Location.Unfound;
        }

        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == OpModeVisionRed.Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == OpModeVisionRed.Location.Right? pixelColor:propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == OpModeVisionRed.Location.Middle? pixelColor:propColor);

        return mat;
    }

}

