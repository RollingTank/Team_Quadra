package Wild;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueCV extends OpenCvPipeline {
    PropPositions propPosition = PropPositions.UNFOUND;
    @Override
    public Mat processFrame(Mat input) {
        Rect left = new Rect(new Point(200, 100), new Point(input.cols()-450,input.rows()-1));
        Rect right = new Rect((new Point(input.cols()-1-550,input.rows()-1-550)), new Point(input.cols()-1,input.rows()-1));
        Mat fresh = input.submat(new Rect(new Point(10,100), new Point(205, 200)));
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Scalar lowHsv = new Scalar(47, 54, 35);
        Scalar highHsv = new Scalar(136, 255, 255);
        Core.inRange(input, lowHsv, highHsv, input);
        Mat leftFrame = input.submat(left);
        Mat rightFrame = input.submat(right);
        int lowerArea = 0, upperArea = Integer.MAX_VALUE-1000;
        double lefterLow = (Core.sumElems(leftFrame).val[0]);
        double lefterHigh = (Core.sumElems(leftFrame).val[0]);
        double centerLow = (Core.sumElems(rightFrame).val[0]);
        double centerHigh = (Core.sumElems(rightFrame).val[0]);

        if(lefterLow >= lowerArea && lefterHigh <= upperArea){
            propPosition = PropPositions.LEFT;
            System.out.println(propPosition);

            //telemetry.addData("Side: ", propPosition);
            Imgproc.cvtColor(fresh, fresh, Imgproc.COLOR_BGR2GRAY);
            return leftFrame;
        }
        else if(centerLow >= lowerArea && centerHigh <= upperArea){
            propPosition = PropPositions.MIDDLE;
            System.out.println(propPosition);
            //telemetry.addData("Side: ", propPosition);
            return rightFrame;
        }
        else{
            propPosition = PropPositions.RIGHT;
            //telemetry.addData("Prop Position: ", propPosition);
            //telemetry.addData("Side: ", propPosition);
            return fresh;
        }


        /*
        if(leftFrame.size().area() >= lowerArea && leftFrame.size().area() <= upperArea){
            propPosition = PropPositions.LEFT;
            System.out.println(propPosition);

            //telemetry.addData("Side: ", propPosition);
            Imgproc.cvtColor(fresh, fresh, Imgproc.COLOR_BGR2GRAY);
            return leftFrame;
        }
        else if(rightFrame.size().area() >= lowerArea && rightFrame.size().area() <= upperArea){
            propPosition = PropPositions.MIDDLE;
            System.out.println(propPosition);
            //telemetry.addData("Side: ", propPosition);
            return rightFrame;
        }
        else{
            propPosition = PropPositions.RIGHT;
            //telemetry.addData("Prop Position: ", propPosition);
            //telemetry.addData("Side: ", propPosition);
            return fresh;
        }
        */


    }
    public enum PropPositions{
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND
    }
}
