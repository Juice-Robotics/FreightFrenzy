package org.firstinspires.ftc.teamcode;



import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;




public class DuckDetectorHSV extends OpenCvPipeline {
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
    static final Scalar WHITE = new Scalar(255, 255, 255);

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


    /*
     * Working variables
     */
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat mat = new Mat();
    Mat workingMatrix = new Mat();
    int avg1;

    static final Rect rectA= new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect rectB = new Rect(
            new Point(140, 35),
            new Point(200, 75));

    static final Rect rectC = new Rect(
            new Point(140, 35),
            new Point(200, 75));


    static final Rect A = new Rect(
            new Point(10, 100),
            new Point(100, 100));
    static final Rect B = new Rect(
            new Point(220, 100),
            new Point(100, 100));

    static final Rect C = new Rect(
            new Point(430, 100),
            new Point(100, 100));

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    // Volatile since accessed by OpMode thread w/o synchronization
   // public volatile AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.FOUR;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */

    public DuckDetectorHSV(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input)
    {
        //inputToCb(input);

        //input.copyTo(workingMatrix);

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(23,50,70);
        Scalar highHSV = new Scalar(32, 255, 255);


        Core.inRange(mat, lowHSV, highHSV, input);



        Mat regionA = mat.submat(A);
        Mat regionB = mat.submat(B);
        Mat regionC = mat.submat(C);

        regionA.release();
        regionB.release();
        regionC.release();


        double leftTotal = Core.sumElems(regionA).val[0]/rectA.area()/255;
        double centerTotal = Core.sumElems(regionB).val[0]/rectB.area()/255;
        double rightTotal = Core.sumElems(regionC).val[0]/rectC.area()/255;

        if (leftTotal > 0.4){
            depositLevel =0;
        }
        else if (centerTotal > 0.4){
            depositLevel =1;
        }
        else {
            depositLevel =2;
        }
       //* avg1 = (int) Core.mean(region1_Cb).val[0];

        telemetry.addData("Left raw value", (int) Core.sumElems(regionA).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(regionB).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(regionC).val[0]);
        telemetry.addData("Left percentage", Math.round(leftTotal * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerTotal * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightTotal * 100) + "%");
        telemetry.addData("depositLevel", depositLevel);

        telemetry.update();



        /*   position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
        if(avg1 > FOUR_RING_THRESHOLD){
            position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.FOUR;
        }else if (avg1 > ONE_RING_THRESHOLD){
            position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.ONE;
        }else{
            position = AutonForwardShoot.SkystoneDeterminationPipeline.RingPosition.NONE;
        }
        */

       // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Imgproc.rectangle(mat, new Rect(10,100,100,100), WHITE, 2);
        Imgproc.rectangle(mat, new Rect(220,100,100,100), WHITE, 2);
        Imgproc.rectangle(mat, new Rect(430,100,100,100), WHITE, 2);

        return mat;
    }

    public int getAnalysis()
    {
        return avg1;
    }
}





