package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ShippingElementDetector extends OpenCvPipeline {
    public enum ElementPosition{
        LEFT,
        MIDDLE,
        RIGHT
    }
    private ElementPosition elementPosition = null;
    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));

    public ShippingElementDetector(Telemetry t) { telemetry = t; } //Constructor

    public ElementPosition getElementPosition(){
        return elementPosition;
    }

    @Override
    public Mat processFrame(Mat input) { //Turn color image to greyscale
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(50, 125, 140);//Color threshhold for yellow
        Scalar highHSV = new Scalar(70, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI); //Create a new image that is a portion of the old one within the rectangle
        Mat middle = mat.submat(MIDDLE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255; //Find % of yellow within each box
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release(); //Free up memory probably
        middle.release();
        right.release();

        telemetry.addData("Percent left:",leftValue*100);
        telemetry.addData("Percent middle:",middleValue*100);
        telemetry.addData("Percent right:",rightValue*100);


        if(leftValue >= middleValue && leftValue >= rightValue){ //Set elementPosition to correct position
            elementPosition = ElementPosition.LEFT;
        }else if (rightValue >= middleValue){
            elementPosition = ElementPosition.RIGHT;
        }else{
            elementPosition = ElementPosition.MIDDLE;
        }

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB); //Convert greyscale back to rgb

        Scalar red = new Scalar(255,0,0); //define red and green vals for rectangles
        Scalar green = new Scalar(0,255,0);

        Imgproc.rectangle(mat, LEFT_ROI, elementPosition==ElementPosition.LEFT ? green : red); //Color each box green or red depending on whether it is the calculated position
        Imgproc.rectangle(mat, MIDDLE_ROI, elementPosition==ElementPosition.MIDDLE ? green : red);
        Imgproc.rectangle(mat, RIGHT_ROI, elementPosition==ElementPosition.RIGHT ? green : red);

        return mat;
    }
}