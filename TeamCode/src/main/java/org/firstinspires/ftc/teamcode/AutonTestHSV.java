package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


enum StateREDWobble{
    START,
    DEPOSITBLOCK,
    MOVETODUCK,
    PARK,
    FORWARDTOLINE,
    STRAFETOSHOOT,
    PRIMESHOOTER,
    SHOOT,
    UNPRIMESHOOTER,
    SECONDLOAD,
    WAITFORLOADCOMPLETE,
    FORWARDTOLINEAGAIN,
    SECONDPRIMESHOOTER,
    SECONDSHOOT,
    DETECTPILE,
    UNPRIMESHOOTERAGAIN,
    MOVE4,
    MOVE1,
    MOVE0,
    DROPWOBBLE,

}

@Autonomous(name="AutonTestHSV", group="Auton Opmode")
public class AutonTestHSV extends LinearOpMode {

    Robot robot;
    boolean first = true;
    long timeChange = 0;
    long lastTime = System.currentTimeMillis();


    private int ringCount = 0;
    public Pose2d startPose = new Pose2d(0, 0, 0);
    private StateBlueWobble currentState;

    private OpenCvCamera webcam;
    private DuckDetector pipeline;
    private DuckDetectorHSV pipelineHSV;
    private DuckDetectPipeline pipelineOG;

    public int depositLevel = 0;

   // currentState = StateBlueWobble.START;

 //   @Override
    /*public void init() {
        robot = new Robot(hardwareMap, true);

        robot.drive.setPoseEstimate(startPose);

        //robot.mpController.updateRequestedPose(0.00001, 0, 0, 0, 0);

       // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());




        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.



    }

    @Override
    public void init_loop(){



        //telemetry.addData("getP", robot.mpController.getP);
      //  telemetry.addData("xCor", robot.xCor);
      //  telemetry.addData("yCor", robot.yCor);
    //    telemetry.addData("rCor", robot.rCor);
    /*    telemetry.addData("tDelta", (System.currentTimeMillis() - timeChange - robot.mpController.initTime));
        telemetry.addData("getVX", (robot.mpController.motionProfileX.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPX", (robot.mpController.motionProfileX.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getVY", (robot.mpController.motionProfileY.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPY", (robot.mpController.motionProfileY.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
*/

       /* telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY()); */





    @Override
    public void runOpMode() throws InterruptedException{


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new DuckDetector(telemetry);

        pipelineHSV= new DuckDetectorHSV(telemetry);

        //webcam.setPipeline(pipeline);

        webcam.setPipeline(pipelineHSV);
        depositLevel = pipelineHSV.depositLevel;
      //  depositLevel = pipeline.depositLevel;

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);







        robot = new Robot(hardwareMap, true);

        robot.drive.setPoseEstimate(startPose);

            //robot.mpController.updateRequestedPose(0, -20, 0, 0, 0);


        //robot.updateLoop();
        if (first) {
                //  timeChange = System.currentTimeMillis() - robot.mpController.initTime;
                first = false;
            }

     /*   telemetry.addData("dTime", System.currentTimeMillis() - lastTime);
        lastTime = System.currentTimeMillis();
      //  telemetry.addData("getP", robot.mpController.getP);
        telemetry.addData("xCor", robot.xCor);
        telemetry.addData("yCor", robot.yCor);
        telemetry.addData("rCor", robot.rCor);
       /* telemetry.addData("tDelta", (System.currentTimeMillis() - timeChange - robot.mpController.initTime));
        telemetry.addData("getVX", (robot.mpController.motionProfileX.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPX", (robot.mpController.motionProfileX.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getVY", (robot.mpController.motionProfileY.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPY", (robot.mpController.motionProfileY.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
*/

       /* telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());


        telemetry.addData("Ring Count", ringCount);*/


            //   telemetry.addData("error", robot.mpController.pidTheta.error);

        /* --Telemetry--
        telemetry.addData("stopped", robot.stopped(true));
        telemetry.addData("PositionY", robot.currentY);
        telemetry.addData("PositionTargetY", robot.targetY);
        telemetry.addData("CorrectionY", robot.correctionY);
        telemetry.addData("highestY", robot.highestY);
        telemetry.addData("PositionX", robot.currentX);
        telemetry.addData("PositionTargetX", robot.targetX);
        telemetry.addData("CorrectionX", robot.correctionX);
        telemetry.addData("highestX", robot.highestX);
        telemetry.addData("PositionTargetX", robot.targetX);
        telemetry.addData("Rotation", robot.currentR);
        telemetry.addData("RotationTarget", robot.targetR);
        telemetry.addData("CorrectionR", robot.correctionR);
         */

            TrajectorySequence depositTraj = robot.drive.trajectorySequenceBuilder(startPose)
                    .turn(Math.toRadians(90))
                    .strafeRight(5)
                    .forward(3)
                    .build();

            TrajectorySequence duckTraj = robot.drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(24)
                    .turn(Math.toRadians(90))
                    .build();


            TrajectorySequence parkTraj = robot.drive.trajectorySequenceBuilder(startPose)
                    .back(20)
                    .turn(Math.toRadians(90))
                    .forward(10)
                    .back(10)
                    .forward(10)
                    .back(10)
                    .forward(10)
                    .build();


            Trajectory trajA = robot.drive.trajectoryBuilder(startPose)
                    .forward(10)
                    .build();

            Trajectory trajB = robot.drive.trajectoryBuilder(startPose)
                    .forward(20)
                    .build();

            Trajectory trajC = robot.drive.trajectoryBuilder(startPose)
                    .forward(30)
                    .build();


            float forwardVal = 0;
            float armVal = 0;
            if (depositLevel == 0) {
                forwardVal = 3;
                armVal = 3;

            } else if (depositLevel == 1) {
                forwardVal = 3;
                armVal = 3;
            } else {

                forwardVal = 3;
                armVal = 3;
            }


            TrajectorySequence masterAuton = robot.drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(3)
                    .turn(Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        robot.carouselOn(4);
                    })
                    .waitSeconds(2)

                    .turn(Math.toRadians(90))
                    .strafeRight(5)
                    .forward(3)

                    .forward(forwardVal)
                    .addTemporalMarker(() -> {

                      //  robot.armOn(3);


                    })

                    .waitSeconds(3)

                    .back(20)
                    .turn(Math.toRadians(90))
                    .forward(10)
                    .back(10)
                    .forward(10)
                    .back(10)
                    .forward(10)
                    .build();









       /* switch(currentState){
            case START:
                robot.intakeOn(4);
                //call open cv stuff
                depositLevel = 2;
                currentState = StateBlueWobble.MOVETODUCK;
                break;
            case MOVETODUCK:



                robot.drive.followTrajectorySequence(duckTraj);
                robot.carouselOn(4);

                currentState = StateBlueWobble.DEPOSITBLOCK;

                break;
            case DEPOSITBLOCK:
                robot.drive.followTrajectorySequence(depositTraj);


                ///using PID control maybe depending on if statemtns which one it is, put in that value, code PID in the arm onclass
                if (depositLevel == 0){
                    robot.drive.followTrajectory(trajA);
                    robot.armOn(1);

                }

                else if (depositLevel == 1){
                    robot.drive.followTrajectory(trajB);
                    robot.armOn(2);
                }
                else{

                    robot.drive.followTrajectory(trajC);
                    robot.armOn(3);
                }



                currentState = StateBlueWobble.PARK;
                break;


            case PARK:

                robot.drive.followTrajectorySequence(parkTraj);
                break;

                */


            //  waitForStart();

            // if (!isStopRequested())

            //  break;

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive())
            {
                /*telemetry.addData("location", pipelineHSV.depositLevel);


                telemetry.update();*/


            }







            /*Trajectory testSpline = robot.drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(10, 10), Math.toRadians(45))
                    .turn(Math.toRadians(90))
                    .build();*/

       /* TrajectorySequence testSpline = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 24, 0))
               // .splineTo(new Vector2d(24, 0), Math.toRadians(0))
                //.splineTo(new Vector2d(0, 24), Math.toRadians(0))
               // .splineTo(new Vector2d(24, 24), Math.toRadians(0))
                //.turn(Math.toRadians(90))
               // .splineTo(new Vector2d(40, 10), Math.toRadians(90))


                .splineTo(new Vector2d(-24, 24), Math.toRadians(0))
                .splineTo(new Vector2d(-5, 24), Math.toRadians(0))
                .splineTo(new Vector2d(-5, 18), Math.toRadians(90))
                // .splineTo(new Vector2d(-10, 48), Math.toRadians(-90))
                .splineTo(new Vector2d(-5,65 ), Math.toRadians(-90))
                .strafeLeft(24)
                .strafeRight(24)
                .strafeLeft(24)
                .strafeRight(24)
                .strafeLeft(24)
                .build();

            robot.drive.followTrajectorySequence(testSpline);*/


    }





}




