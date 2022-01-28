package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DuckDetector extends OpenCvPipeline {
    /*
     * An enum to define the skystone position
     */

    Telemetry telemetry;
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }

    public int depositLevel;
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    //CHANGE TO TUNE
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90,90);

    static final int REGION_WIDTH = 70;
    static final int REGION_HEIGHT = 50;

    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 135;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    static final Rect A = new Rect(10,100,100,100);
    static final Rect B = new Rect(
         220,100,100,100);

    static final Rect C = new Rect(
           430,100,100,100);



    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat mat = new Mat();
    Mat workingMatrix = new Mat();
    int avg1;

    // Volatile since accessed by OpMode thread w/o synchronization
   // public volatile AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.FOUR;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    public DuckDetector(Telemetry t) { telemetry = t; }
    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);

        //input.copyTo(workingMatrix);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);






        Mat regionA = input.submat(A);
        Mat regionB = input.submat(B);
        Mat regionC = input.submat(C);
       /* Mat regionC = workingMatrix.submat(20, 10,10,10);
        Mat regionE = workingMatrix.submat(20, 10,10,10);*/



        double leftTotal = Core.sumElems(regionA).val[2];
        double centerTotal = Core.sumElems(regionB).val[2];
        double rightTotal = Core.sumElems(regionC).val[2];

        if (leftTotal < rightTotal){


            if (leftTotal < centerTotal){
                depositLevel = 0;
            }
            else{

                depositLevel = 1;
            }
        }
        else {

            if (centerTotal < rightTotal) {

                depositLevel = 1;
            }
            else {
                depositLevel = 2;
            }
        }
       //* avg1 = (int) Core.mean(region1_Cb).val[0];



        telemetry.addData("Left ", leftTotal);
        telemetry.addData("Center ", centerTotal);
        telemetry.addData("Right ", rightTotal);
        telemetry.addData("depositLevel", depositLevel);

        telemetry.update();



        Imgproc.rectangle(mat, A, BLUE, 2);
        Imgproc.rectangle(mat, B, RED, 2);
        Imgproc.rectangle(mat, C, GREEN, 2);



        //return input;
        return mat;
    }

    public int getAnalysis()
    {
        return depositLevel;
    }
}





