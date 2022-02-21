package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.time.temporal.ValueRange;




@Autonomous(name="autonRedPark", group="Auton Opmode")
public class autonRedPark extends LinearOpMode {

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

    public double armVal = 0;

    public float forwardVal = 0;



    @Override
    public void runOpMode() throws InterruptedException{


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new DuckDetector(telemetry);

        pipelineHSV= new DuckDetectorHSV(telemetry);

        webcam.setPipeline(pipeline);

        //webcam.setPipeline(pipelineHSV);
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

        webcam.stopStreaming();



        robot = new Robot(hardwareMap, true);

       /* dashboard.addConfigVariable("PIDController", "CarouselPID", spinmotorPID);
        dashboard.addConfigVariable("PIDController", "armPID", robot.v4bArm.armPID);
        dashboard.addConfigVariable("PIDController", "armPID2", robot.v4bArm.armPID2);*/


        robot.drive.setPoseEstimate(startPose);


        robot.updateLoop();

        telemetry.addData("targetCarouselRPM", robot.carousel.targetRPM);
        telemetry.addData("currentCarouselRPM", robot.carousel.currentRPM);
        telemetry.addData("targetV42BDistance", robot.v4bArm.targetDistance);
        telemetry.addData("currentV4BDistance", robot.v4bArm.currentDistance);







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


        waitForStart();

        if (isStopRequested()) return;





        TrajectorySequence autonBlueDeposit = robot.drive.trajectorySequenceBuilder(new Pose2d(-34, 65, 0))
                .splineTo(new Vector2d(-48, 65), Math.toRadians(0))

               // .addDisplacementMarker(() -> {
                    // robot.carousel.start(20);



              //  })
                .waitSeconds(2)
               // .addDisplacementMarker(() -> {
                    //robot.carousel.start(40);

                //})
                .splineTo(new Vector2d(-10, 48), Math.toRadians(0))
                .splineTo(new Vector2d(-10, 36), Math.toRadians(90))

                .forward(forwardVal)
               // .addTemporalMarker(() -> {


                    //      robot.v4bArm.start(armVal);
                    //     robot.depositor.onClick(true);

                    /// maybe make a reset method


                //})

                .build();



        TrajectorySequence autonBluePark = robot.drive.trajectorySequenceBuilder(new Pose2d(0, 24, 0))
                .splineTo(new Vector2d(-10, 65), Math.toRadians(-90))
                .strafeLeft(48)
                .strafeRight(24)
                .strafeLeft(24)
                .strafeRight(24)
                .strafeLeft(56)
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


        TrajectorySequence autonRedPark = robot.drive.trajectorySequenceBuilder(startPose)
                //.splineTo(new Vector2d(-10, -65), Math.toRadians(-90))
               /* .splineTo(new Vector2d(10, 48), Math.toRadians(0))
                .splineTo(new Vector2d(10, 36), Math.toRadians(90))*/
                .strafeRight(24)
                .forward(8)
                .back(8)
                .strafeLeft(36)
                .turn(Math.toRadians(90))
               /* .addTemporalMarker(() -> {

                    robot.carousel.run(-3000);
                    sleep(1500);
                    robot.carousel.stop();
            //   robot.v4bArm.start(armVal);
            //   robot.depositor.onClick(true);

            /// maybe make a reset method


                })*/
                .strafeLeft(24)
                .build();


       // robot.drive.followTrajectorySequence(autonRedPark);

        robot.v4bArm.work(0.7f,700);
      //  robot.v4bArm.resetAllEncoders();
      //  robot.v4bArm.stop();
       /* robot.carousel.run(-3000);
        sleep(1500);
        robot.carousel.stop();*/

        telemetry.addData("taegetDistance", robot.v4bArm.targetDistance);
        telemetry.addData("currentDistance", robot.v4bArm.armMotor1.getEncoderValue());

    }





}




