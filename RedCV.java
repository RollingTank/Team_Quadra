package Wild;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedCV extends OpenCvPipeline {
    PropPositions propPosition = PropPositions.UNFOUND;
    @Override
    public Mat processFrame(Mat input) {
        Rect left = new Rect(new Point(10, 100), new Point(105,200));
        Rect right = new Rect(new Point(120, 100), new Point(205,200));
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Scalar lowHsv = new Scalar(1.8, 96, 25);
        Scalar highHsv = new Scalar(30, 255, 255);
        Core.inRange(input, lowHsv, highHsv, input);
        Mat leftFrame = input.submat(left);
        Mat rightFrame = input.submat(right);
        int lowerArea = 400, upperArea = 500;
        double lefterLow = (Core.sumElems(leftFrame).val[0] / left.area() / 255) - 50;
        double lefterHigh = (Core.sumElems(leftFrame).val[0] / left.area() / 255) - 50;
        double centerLow = (Core.sumElems(leftFrame).val[0] / right.area() / 255) - 50;
        double centerHigh = (Core.sumElems(leftFrame).val[0] / right.area() / 255) - 50;
        if(leftFrame.size().area() >= lowerArea && leftFrame.size().area() <= upperArea){
            propPosition = PropPositions.LEFT;
            System.out.println(propPosition);
            return input;
        }
        else if(rightFrame.size().area() >= lowerArea && rightFrame.size().area() <= upperArea){
            propPosition = PropPositions.LEFT;
            System.out.println(propPosition);
            return input;
        }
        else{
            propPosition = PropPositions.MIDDLE;
            System.out.println(propPosition);
            return input;
        }


    }
    public enum PropPositions{
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}
