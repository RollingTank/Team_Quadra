//package Wild;
package org.firstinspires.ftc.teamcode.common.vision;

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

public class openBLUEFar extends OpenCvPipeline {
    public static boolean DETECT_RED = false;
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
        Middle
    };
    private Location location;

    static final Rect Left_ROI = new Rect(new Point(0,400), new Point(250,150));
    static final Rect Middle_ROI = new Rect(new Point(350,400), new Point(600,160));
    static final Rect Right_ROI = new Rect(new Point(650, 400), new Point(950,150));

    public void CenterStageCVDetection(Telemetry t){
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE_LOW = new Scalar(MINIMUM_BLUE_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE_LOW = new Scalar(MAXIMUM_BLUE_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_BLUE_HIGH = new Scalar(MINIMUM_BLUE_HIGH_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE_HIGH = new Scalar(MAXIMUM_BLUE_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);

        if (!DETECT_RED) {
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_BLUE_LOW, MAXIMUM_BLUE_LOW, mat1);
            Core.inRange(mat2, MINIMUM_BLUE_HIGH, MAXIMUM_BLUE_HIGH, mat2);
            Core.bitwise_or(mat1,mat2,mat);
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

        double leftValue = Core.sumElems(left).val[0];
        double rightValue = Core.sumElems(right).val[0];
        double middleValue = Core.sumElems(middle).val[0];

        //telemetry.addData("left raw val:", leftValue);
        //telemetry.addData("right raw val:", rightValue);
        //telemetry.addData("middle raw val:", middleValue);

        left.release();
        right.release();
        middle.release();

        if (leftValue >= rightValue && leftValue >= middleValue) {
            location = Location.Left;
            //telemetry.addData("location:", "left");
        } else if (rightValue >= middleValue) {
            location = Location.Right;
            //telemetry.addData("location:", "right");
        } else {
            location = Location.Middle;
            //telemetry.addData("location:", "middle");
        }

        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == Location.Right? pixelColor:propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == Location.Middle? pixelColor:propColor);

        return mat;
    }
}

