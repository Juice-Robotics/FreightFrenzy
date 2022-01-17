package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Robot;




enum StateBlueWobble{
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

@TeleOp(name="AutonTest2", group="Auton Opmode")
public class AutonTest2 extends OpMode {

    Robot robot;
    boolean first = true;
    long timeChange = 0;
    long lastTime = System.currentTimeMillis();


    private int ringCount = 0;
    public Pose2d startPose = new Pose2d(0, 0, 0);
    private StateBlueWobble currentState;

    public int depositLevel = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);

        robot.drive.setPoseEstimate(startPose);

        //robot.mpController.updateRequestedPose(0.00001, 0, 0, 0, 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());




        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.



    }

    @Override
    public void init_loop(){

        currentState = StateBlueWobble.START;

        //telemetry.addData("getP", robot.mpController.getP);
        telemetry.addData("xCor", robot.xCor);
        telemetry.addData("yCor", robot.yCor);
        telemetry.addData("rCor", robot.rCor);
    /*    telemetry.addData("tDelta", (System.currentTimeMillis() - timeChange - robot.mpController.initTime));
        telemetry.addData("getVX", (robot.mpController.motionProfileX.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPX", (robot.mpController.motionProfileX.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getVY", (robot.mpController.motionProfileY.getV(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
        telemetry.addData("getPY", (robot.mpController.motionProfileY.getP(System.currentTimeMillis() - timeChange - robot.mpController.initTime)));
*/

        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());



    }

    @Override
    public void loop() {

        //robot.mpController.updateRequestedPose(0, -20, 0, 0, 0);
        robot.drive.setPoseEstimate(startPose);

        robot.updateLoop();
        if (first) {
          //  timeChange = System.currentTimeMillis() - robot.mpController.initTime;
            first = false;
        }

        telemetry.addData("dTime", System.currentTimeMillis() - lastTime);
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

        telemetry.addData("xVel", robot.robotPose.getXVelocity());
        telemetry.addData("yVel", robot.robotPose.getYVelocity());
        telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
        telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

        telemetry.addData("rot", robot.robotPose.getHeading());
        telemetry.addData("x", robot.robotPose.getX());
        telemetry.addData("y", robot.robotPose.getY());


        telemetry.addData("Ring Count", ringCount);
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
                .strafeLeft(3)
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
        if (depositLevel == 0){
            forwardVal = 3;
            armVal= 3;

        }

        else if (depositLevel == 1) {
            forwardVal = 3;
            armVal = 3;
        }
        else{

            forwardVal = 3;
            armVal= 3;
        }


        TrajectorySequence masterAuton = robot.drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(3)
                .turn(Math.toRadians(90))
                .addDisplacementMarker(()->{
                    robot.carouselOn(4);
                })
                .waitSeconds(2)

                .turn(Math.toRadians(90))
                .strafeRight(5)
                .forward(3)

                .forward(forwardVal)
                .addTemporalMarker(()->{

                    robot.armOn(3);


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





       // waitForStart();
      //  if (!isStopRequested())
        switch(currentState){
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


            //    waitForStart();

            //    if (!isStopRequested())
                   // robot.drive.followTrajectorySequence(trajSeq);
              //  break;


        }





}

}


