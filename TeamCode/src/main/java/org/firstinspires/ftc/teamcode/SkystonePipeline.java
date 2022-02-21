package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystonePipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        CENTER
    }
    private Location location;

    public int position = 0;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar WHITE = new Scalar(255, 255, 255);

    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    static final Rect A = new Rect(
            new Point(60, 15),
            new Point(120, 35));
    static final Rect B = new Rect(
            new Point(140, 15),
            new Point(200, 35));

    static final Rect C = new Rect(
            new Point(220, 15),
            new Point(280, 35));


    static final Rect D = new Rect(30,120,100,100);
    static final Rect E = new Rect(240,120,100,100);

    static final Rect F = new Rect(450,120,100,100);





    public SkystonePipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(D);
        Mat center = mat.submat(E);
        Mat right = mat.submat(F);

        double leftValue = Core.sumElems(left).val[0] / D.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / E.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / F.area() / 255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        if (leftValue > 0.1){
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");

            position = 0;
        }
        else if (centerValue > 0.1){
            location = Location.CENTER;
            telemetry.addData("Skystone Location", "center");
            position = 1;
        }
        else {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
            position = 2;
        }

       /* boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }*/

        telemetry.addData("Skystone Location", position);
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

      /*Imgproc.rectangle(mat, A, position==0? colorSkystone:colorStone);
        Imgproc.rectangle(mat, B, position==1? colorSkystone:colorStone);
        Imgproc.rectangle(mat, C, position==2? colorSkystone:colorStone);*/

        Imgproc.rectangle(mat, D, GREEN, 2);
        Imgproc.rectangle(mat, E, BLUE, 2);
        Imgproc.rectangle(mat, F, WHITE, 2);

       /* Imgproc.rectangle(mat, A, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, B, location == Location.NOT_FOUND? colorSkystone:colorStone);
        Imgproc.rectangle(mat, C, location == Location.RIGHT? colorSkystone:colorStone);*/

        return mat;
    }

    public Location getLocation() {
        return location;
    }

    public int getPosition(){

        return position;
    }
}
