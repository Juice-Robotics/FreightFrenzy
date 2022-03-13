package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config

@Autonomous(name="autonRedParkWarehouse2", group="Auton Opmode")
public class autonRedParkWarehouse2 extends LinearOpMode {

    Robot robot;

    boolean first = true;
    long timeChange = 0;
    long lastTime = System.currentTimeMillis();


    private int ringCount = 0;
    public Pose2d startPose = new Pose2d(0, 0, 0);
    private StateBlueWobble currentState;

    private OpenCvCamera webcam;
    private SkystonePipeline pipeline;
    private DuckDetectorHSV pipelineHSV;
    private DuckDetectPipeline pipelineOG;

    public int depositLevel = 0;

    public double armVal = 0;

    public float forwardVal = 0;

    public int go = 0;

    boolean bottom = false;

    public static PIDController armPIDdisplay = new PIDController(0,0,0,0,false);




    @Override
    public void runOpMode() throws InterruptedException{


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new SkystonePipeline(telemetry);

        pipelineHSV = new DuckDetectorHSV(telemetry);

        webcam.setPipeline(pipeline);

        //webcam.setPipeline(pipelineHSV);

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


        depositLevel = pipeline.getPosition();

        telemetry.addData("Positon", depositLevel);

        // webcam.stopStreaming();



        robot = new Robot(hardwareMap, true);
        robot.v4bArm.resetAllEncoders();
        robot.carousel.resetAllEncoders();





       /* dashboard.addConfigVariable("PIDController", "CarouselPID", spinmotorPID);
        dashboard.addConfigVariable("PIDController", "armPID", robot.v4bArm.armPID);
        dashboard.addConfigVariable("PIDController", "armPID2", robot.v4bArm.armPID2);*/


        robot.drive.setPoseEstimate(startPose);


        robot.updateLoop();

        telemetry.addData("targetCarouselRPM", robot.carousel.targetRPM);
        telemetry.addData("currentCarouselRPM", robot.carousel.currentRPM);
        telemetry.addData("targetV42BDistance", robot.v4bArm.targetDistance);
        telemetry.addData("currentV4BDistance", robot.v4bArm.currentDistance);



        //755, 850


        //9, 7, 8
        if (depositLevel == 2) {

            forwardVal = 7;
            armVal = 975;
            bottom = true;

        } else if (depositLevel == 1) {
            forwardVal = 6;
            armVal = 790;
        } else {


            forwardVal = 9;
            armVal = 685;

        }

        //  forwardVal = 5;

        //  armVal = 850;

        waitForStart();

        depositLevel = pipeline.getPosition();

        telemetry.addData("Positon", depositLevel);

        if (depositLevel == 2) {

            forwardVal = 7;
            armVal = 975;
            bottom = true;

        } else if (depositLevel == 1) {
            forwardVal = 6;
            armVal = 790;
        } else {


            forwardVal = 9;
            armVal = 685;

        }

        if (isStopRequested()) return;


        while (opModeIsActive() && !isStopRequested()) {

            robot.updateLoop();

            TrajectorySequence currentPlan = robot.drive.trajectorySequenceBuilder(startPose)
                    //.splineTo(new Vector2d(-10, -65), Math.toRadians(-90))
                    /* .splineTo(new Vector2d(10, 48), Math.toRadians(0))
                     .splineTo(new Vector2d(10, 36), Math.toRadians(90))*/
                    .waitSeconds(5)
                    .strafeLeft(30)

                    .build();

            TrajectorySequence planPart2 = robot.drive.trajectorySequenceBuilder(startPose)
                    .forward(forwardVal)

                    //add marker to keep arm up
                    .build();
            TrajectorySequence planPart3 = robot.drive.trajectorySequenceBuilder(startPose)
                    .waitSeconds(1.5)
                    .back(forwardVal)
                    .addTemporalMarker(1.5, () -> {
                // This marker runs two seconds into the trajectory

                // Run your action in here!

                        robot.depositor.inTake();

                        if (robot.v4bArm.armMotor1.getEncoderValue() > 50){

                            robot.v4bArm.reverse(100);
                        }

                        else {
                            robot.v4bArm.stop();

                        }



                     })

                    //.strafeRight(44)*/
                    .build();

            //og was 100
            TrajectorySequence turnPlease = robot.drive.trajectorySequenceBuilder(startPose)
                    .turn(Math.toRadians(90))
                    .strafeLeft(12)
                    //.back(6)
                    .back(34)
                    .build();


            TrajectorySequence planPart4 = robot.drive.trajectorySequenceBuilder(startPose)
                    .strafeLeft(28)
                    .turn(Math.toRadians(-15))
                    .back(4)
                    .build();



            telemetry.addData("currentRPM", robot.carousel.currentRPM);

            telemetry.addData("currentTime", System.currentTimeMillis());


            telemetry.addData("taegetDistance", robot.v4bArm.targetDistance);
            telemetry.addData("currentDistance", robot.v4bArm.armMotor1.getEncoderValue());






            if(go == 0){

                // robot.drive.followTrajectorySequence(turnPlease);



                robot.drive.followTrajectorySequence(currentPlan);
                go=1;
            }


            else if (go == 1) {

                if(bottom) {



                    //og 755
                    if (robot.v4bArm.armMotor1.getEncoderValue() < armVal) {
                        //robot.v4bArm.work(0.5f,100);
                        robot.v4bArm.start(armVal);


                    } else {

                        robot.v4bArm.stop();
                        go = 2;

                    }
                }
                else{

                    robot.drive.followTrajectorySequence(planPart2);
                    go = 2;

                }



            }

            else if (go == 2) {

                if(bottom){
                    robot.drive.followTrajectorySequence(planPart2);
                    go=3;



                }
                else{

                    if (robot.v4bArm.armMotor1.getEncoderValue() < armVal) {
                        //robot.v4bArm.work(0.5f,100);
                        robot.v4bArm.start(armVal);


                    } else {


                        robot.v4bArm.stop();

                        go=3;

                    }

                }


            }

            else if (go == 3) {


                if ( (robot.v4bArm.armMotor1.getEncoderValue() > armVal+10) && (robot.v4bArm.armMotor1.getEncoderValue() < armVal-10) ){
                    // if ( robot.v4bArm.armMotor1.getEncoderValue() > armVal){


                    // double difference = last-robot.v4bArm.armMotor1.getEncoderValue();


                    robot.v4bArm.reverse(1000);

                }
                else {

                    robot.v4bArm.stop();

                }




                robot.depositor.outTake();

                /*if (robot.v4bArm.armMotor1.getEncoderValue() > 755){

                    robot.v4bArm.reverse(10);

                }
                else {

                    robot.v4bArm.stop();

                }*/

                go = 4;
            }

            else if (go == 4){



                robot.drive.followTrajectorySequence(planPart3);

            //    robot.drive.followTrajectorySequence(turnPlease);

                go=5;
            }



         /*   else if (go == 5) {

                if (robot.carousel.carousel.getEncoderValue() > -600) {

                    robot.carousel.start(-3000);





                }
                else {
                    go=6;
                    robot.carousel.stop();
                }



            }*/



            else if (go == 5) {
               // robot.drive.followTrajectorySequence(planPart4);
                robot.drive.followTrajectorySequence(turnPlease);
                go=6;
            }

           /* else if (go == 7) {


                robot.depositor.inTake();

                /*if (robot.v4bArm.armMotor1.getEncoderValue() > 755){

                    robot.v4bArm.reverse(10);

                }
                else {

                    robot.v4bArm.stop();

                }*/

           /*     go = 8;
            }

            else if (go == 8){

                //  double currentDistance = robot.v4bArm.armMotor1.getEncoderValue();
                if (robot.v4bArm.armMotor1.getEncoderValue() > 0){

                    robot.v4bArm.reverse(100);
                }

                else {
                    robot.v4bArm.stop();
                    go = 9;
                }



            }

           /* else if (robot.v4bArm.armShift==false && robot.v4bArm.armMove==false){



                if (robot.v4bArm.armMotor1.getEncoderValue() > last){

                    robot.v4bArm.reverse(1000);

                }
                else {

                    robot.v4bArm.stop();

                }

            }*/
            else{
                telemetry.addData("message", "AUTON FINISHED");

            }








        }


    }


}




