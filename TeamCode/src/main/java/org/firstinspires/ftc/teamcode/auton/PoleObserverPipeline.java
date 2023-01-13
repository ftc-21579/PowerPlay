package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PoleObserverPipeline extends OpenCvPipeline{
    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    private String position = "none";

    Mat leftCrop, centerCrop, rightCrop;
    double leftavgfin, centeravgfin, rightavgfin;

    Scalar rectColor = new Scalar(255.0, 0.0, 0.0);

    Telemetry telemetry;

    public PoleObserverPipeline(Telemetry telemetry) {
        frameList = new ArrayList<>();
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        // Start custom code

        Rect leftRect = new Rect(1, 1, 519, 719);
        Rect centerRect = new Rect(520, 1, 220, 719);
        Rect rightRect = new Rect(740, 1, 540, 719);

        Imgproc.rectangle(finalMask, leftRect, rectColor, 2);
        Imgproc.rectangle(finalMask, centerRect, rectColor, 2);
        Imgproc.rectangle(finalMask, rightRect, rectColor, 2);

        leftCrop = finalMask.submat(leftRect);
        centerCrop = finalMask.submat(centerRect);
        rightCrop = finalMask.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(centerCrop, centerCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar centeravg = Core.mean(centerCrop);
        Scalar rightavg = Core.mean(rightCrop);

        leftavgfin = leftavg.val[0];
        centeravgfin = centeravg.val[0];
        rightavgfin = rightavg.val[0];

        if (centeravgfin > leftavgfin && centeravgfin > rightavgfin) {
            position = "center";
            telemetry.addData("Position", "centered");
        } else if (leftavgfin > rightavgfin && leftavgfin > centeravgfin) {
            position = "left";
            telemetry.addData("Position", "left");
        } else if (rightavgfin > leftavgfin && rightavgfin > centeravgfin) {
            position = "right";
            telemetry.addData("Position", "right");
        } else {
            position = "none";
            telemetry.addData("Position", "NOT FOUND");
        }

        // End Custom Code

        telemetry.update();

        //release all the data
        //input.release();
        Imgproc.cvtColor(finalMask, input, Imgproc.COLOR_HSV2RGB);
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        mat.release(); // First Mat
        masked.release();
        edges.release();
        thresh.release();
        //finalMask.release();
        //change the return to whatever mat to see from the above mats. Ensure it does not get released, and whatever was there first does get released

        return finalMask;
    }


    public String getPosition() {return(position);}
}
