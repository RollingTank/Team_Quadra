package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

public class JJ implements CameraStreamSource, VisionProcessor {

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
        Middle
    };
    public openREDClose.Location location;

    static final Rect Left_ROI = new Rect(new Point(0,400), new Point(250,150));
    static final Rect Middle_ROI = new Rect(new Point(400,400), new Point(600,160));
    static final Rect Right_ROI = new Rect(new Point(700, 400), new Point(950,150));
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public void CenterStageCVDetection(Telemetry t){
        telemetry = t;
    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        linePaint = new Paint();
        linePaint.setColor(Color.WHITE);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(12);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);

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

        if (leftValue >= rightValue && leftValue >= middleValue) {
            location = openREDClose.Location.Left;
            //telemetry.addData("location:", "left");
        } else if (rightValue >= middleValue) {
            location = openREDClose.Location.Right;
            //telemetry.addData("location:", "right");
        } else {
            location = openREDClose.Location.Middle;
            //telemetry.addData("location:", "middle");
        }

        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == openREDClose.Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == openREDClose.Location.Right? pixelColor:propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == openREDClose.Location.Middle? pixelColor:propColor);

        return mat;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        switch (location){
            case Left:
                canvas.drawLine(Left_ROI.x, Left_ROI.y, Left_ROI.x+Left_ROI.width, Left_ROI.y+Left_ROI.height, linePaint);
            case Middle:
                canvas.drawLine(Middle_ROI.x, Middle_ROI.y, Middle_ROI.x+Middle_ROI.width, Middle_ROI.y+Left_ROI.height, linePaint);
            case Right:
                canvas.drawLine(Right_ROI.x, Right_ROI.y, Right_ROI.x+Right_ROI.width, Right_ROI.y+Right_ROI.height, linePaint);
        }}


}

