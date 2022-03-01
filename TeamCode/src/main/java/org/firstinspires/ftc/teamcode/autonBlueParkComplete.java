package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="autonBlueParkComplete", group="Auton Opmode")
public class autonBlueParkComplete extends OpMode {

    Robot robot;

    boolean first = true;
    long timeChange = 0;
    long lastTime = System.currentTimeMillis();


    private long delayTime = 0l;

    FtcDashboard dashboard;
    public static double targetRPM;
    public static double currentRPM;
    public static PIDController  carouselPID;
    private int ringCount = 0;
    public Pose2d startPose = new Pose2d(0, 0, 0);
    private StateBlueWobble currentState;

    private OpenCvCamera webcam;
    private DuckDetector pipeline;
    private DuckDetectorHSV pipelineHSV;
    private DuckDetectPipeline pipelineOG;

    public int depositLevel = 0;

    public double armVal = 0;

    public float forwardVal = 0;

    public int go = 0;

    public static PIDController armPID;
    public static PIDController armPID2;
    public static PIDController spinmtorPID;





    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new DuckDetector(telemetry);

        pipelineHSV = new DuckDetectorHSV(telemetry);

        SkystonePipeline pipey = new SkystonePipeline(telemetry);

        webcam.setPipeline(pipey);

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

       // webcam.stopStreaming();


        robot = new Robot(hardwareMap, true);

        depositLevel = pipey.getPosition();

        targetRPM = robot.carousel.targetRPM;
        currentRPM = robot.carousel.currentRPM;
       // carouselPID  = robot.carousel.spinmotorPID;

       /* dashboard.addConfigVariable("PIDController", "CarouselPID", spinmotorPID);
        dashboard.addConfigVariable("PIDController", "armPID", robot.v4bArm.armPID);
        dashboard.addConfigVariable("PIDController", "armPID2", robot.v4bArm.armPID2);*/


        robot.drive.setPoseEstimate(startPose);


        robot.updateLoop();

        robot.v4bArm.resetAllEncoders();
        robot.carousel.resetAllEncoders();

        robot.v4bArm.targetDistance = 1000;

        telemetry.addData("targetCarouselRPM", robot.carousel.targetRPM);
        telemetry.addData("currentCarouselRPM", robot.carousel.currentRPM);
        telemetry.addData("targetV42BDistance", robot.v4bArm.targetDistance);
        telemetry.addData("currentV4BDistance", robot.v4bArm.currentDistance);




    }

    public void init_loop() {

        if (depositLevel == 0) {
            forwardVal = 8;
            armVal = 755;

        } else if (depositLevel == 1) {
            forwardVal = 7;
            armVal = 950;
        } else {

            forwardVal = 6;
            armVal = 1060;
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

                    // robot.armOn(3);


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





        TrajectorySequence autonBlueDeposit = robot.drive.trajectorySequenceBuilder(new Pose2d(-34, 56, 0))
                .splineTo(new Vector2d(-48, 65), Math.toRadians(0))

                .addDisplacementMarker(() -> {
                    // robot.carousel.start(20);


                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    //robot.carousel.start(40);

                })
                .splineTo(new Vector2d(-10, 48), Math.toRadians(0))
                .splineTo(new Vector2d(-10, 36), Math.toRadians(90))

                .forward(forwardVal)
                .addTemporalMarker(() -> {


                    //      robot.v4bArm.start(armVal);
                    //     robot.depositor.onClick(true);

                    /// maybe make a reset method


                })

                .build();


        TrajectorySequence autonBluePark = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 24, 0))
                .forward(forwardVal)
                .addDisplacementMarker(() -> {


                    while (robot.v4bArm.armMotor1.getEncoderValue() < armVal) {

                        robot.v4bArm.armMotor1.setSpeed(0.4f);
                        robot.v4bArm.armMotor2.setSpeed(0.4f);


                    }
                    robot.depositor.outTake();


                })
                .build();


        TrajectorySequence autonRedDeposit = robot.drive.trajectorySequenceBuilder(new Pose2d(-34, -56, 0))

                .splineTo(new Vector2d(-48, -65), Math.toRadians(0))

                .addDisplacementMarker(() -> {
                    ///  robot.carousel.start(20);


                })
                .waitSeconds(2)
                .addDisplacementMarker(() -> {
                    //robot.carousel.start(40);

                })
                .splineTo(new Vector2d(-10, -48), Math.toRadians(0))
                .splineTo(new Vector2d(-10, -36), Math.toRadians(90))

                .forward(forwardVal)
                .addTemporalMarker(() -> {


                    //   robot.v4bArm.start(armVal);
                    //   robot.depositor.onClick(true);

                    /// maybe make a reset method


                })

                .build();





    }

    public void loop() {

        robot.updateLoop();

        //long time = System.currentTimeMillis();


        TrajectorySequence currentPlan = robot.drive.trajectorySequenceBuilder(startPose)
                //.splineTo(new Vector2d(-10, -65), Math.toRadians(-90))
                /* .splineTo(new Vector2d(10, 48), Math.toRadians(0))
                 .splineTo(new Vector2d(10, 36), Math.toRadians(90))*/
                .strafeRight(30)

                .build();

        TrajectorySequence planPart2 = robot.drive.trajectorySequenceBuilder(startPose).forward(forwardVal)
                .build();
        TrajectorySequence planPart3 = robot.drive.trajectorySequenceBuilder(startPose).back(forwardVal)
                .strafeLeft(46)
                .build();

        TrajectorySequence turnPlease = robot.drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(90))
                .build();



        TrajectorySequence planPart4 = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(24)
                .build();



       /* TrajectorySequence autonRedPark = robot.drive.trajectorySequenceBuilder(new Pose2d(-34, 65, 0))
                //.splineTo(new Vector2d(-10, -65), Math.toRadians(-90))

                .splineTo(new Vector2d(10, 48), Math.toRadians(0))
                .splineTo(new Vector2d(10, 36), Math.toRadians(90))
                //.addTemporalMarker(10, () -> {

                   // robot.carousel.start(5000);


                    //   robot.v4bArm.start(armVal);
                    //   robot.depositor.onClick(true);

                    /// maybe make a reset method


               // })
              //  .strafeRight(10)
                .build();*/




       /* if (go == 0){
            robot.drive.followTrajectorySequence(autonRedPark);
            go = 1;

        }*/


        // lastTime = time;

        //robot.v4bArm.move(100, 1000);

        long startTimer = System.currentTimeMillis(); // t = 0


       /* if((System.currentTimeMillis() - startTimer )> 0 && System.currentTimeMillis() - startTimer < 30000){
            robot.carousel.start(5000);
        } else {
            robot.carousel.stop();
        }*/

       /* robot.carousel.start(5000);
        //Thread.sleep(3000);
        robot.carousel.stop();*/


        /* robot.carousel.start(100);*/

        // robot.carousel.start(1000);

        telemetry.addData("currentRPM", robot.carousel.currentRPM);
        telemetry.addData("startTime", startTimer);
        telemetry.addData("currentTime", System.currentTimeMillis());


        telemetry.addData("taegetDistance", robot.v4bArm.targetDistance);
        telemetry.addData("currentDistance", robot.v4bArm.armMotor1.getEncoderValue());




        if (go == 0){



            robot.drive.followTrajectorySequence(turnPlease);


            go=1;
        }

       else if (go == 1) {

            if (robot.carousel.carousel.getEncoderValue() < 500) {

                robot.carousel.start(3000);





            }
            else {
                go=2;
                robot.carousel.stop();
            }



         }



        else if (go == 2) {
             robot.drive.followTrajectorySequence(planPart4);
            go=3;
        }

       else{
            telemetry.addData("message", "AUTON FINISHED");

        }





    }


}

