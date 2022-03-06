package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This opmode assumes you have your own robot class and simply wish to utilize Road Runner's
 * packaged localizer tools.
 */
@TeleOp(group = "advanced")
public class DriverControl extends LinearOpMode {


    boolean changed = false;

    public boolean previousArmToggle = false;
    public boolean armEnabled= false;
    private boolean intakeRetract = false;
    private boolean intakeOnForward = false;
    private boolean intakeOnReverse = false;
    int armOn = 0;

    PIDController armPID3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        // This is assuming you are using StandardTrackingWheelLocalizer.java
        // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your
        // configuration differs
      //  StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
      //  myLocalizer.setPoseEstimate(PoseStorage.currentPose);


        robot.v4bArm.resetAllEncoders();
        //robot.carousel.resetAllEncoders();



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {




            long startTime = System.currentTimeMillis();


            //robot.updateLoop();
             robot.v4bArm.updateDistance();


            // Make sure to call myLocalizer.update() on *every* loop
            // Increasing loop time by utilizing bulk reads and minimizing writes will increase your
            // odometry accuracy
          //  myLocalizer.update();


            //robot.updateLoop(); //DISABLE FOR COMPETITION

            // Retrieve your pose
          //  Pose2d myPose = myLocalizer.getPoseEstimate();

            // Print your pose to telemetry
          //  telemetry.addData("x", myPose.getX());
          //  telemetry.addData("y", myPose.getY());
          //  telemetry.addData("heading", myPose.getHeading());
            telemetry.update();




            // Teleop driving part
            // Mecanum example code from gm0
            // https://gm0.org/en/stable/docs/software/mecanum-drive.html
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            robot.toggleIntake(gamepad1.a);
            robot.intakeOn(gamepad1.right_trigger,startTime);

            robot.intakeReverse(gamepad1.left_trigger);

         //   robot.armOn(gamepad1.b);

          //  robot.toggleDeposit(gamepad1.b);
            robot.toggleDeposit2(gamepad1.y);
            robot.toggleCarousel(gamepad1.x);

           robot.moveLift(gamepad2.left_trigger, gamepad2.right_trigger);

         //  robot.armPreset(gamepad2
           robot.armBottom(gamepad1.left_bumper);
           robot.armMiddle(gamepad1.dpad_left);
           robot.armTop(gamepad1.dpad_right);
           robot.armRetract(gamepad1.dpad_down);


           double last = robot.v4bArm.armMotor1.getEncoderValue();


           telemetry.addData("spinDistance", robot.carousel.carousel.getEncoderValue());

           telemetry.addData("armDistance", robot.v4bArm.armMotor1.getEncoderValue());




           telemetry.addData("armOn", robot.armOn);


           /* if (gamepad1.left_trigger >= 0.5f){
                robot.intake.start();
                intakeOnForward = true;
                intakeRetract = true;
            } else if (!intakeOnReverse){


                // intake.stop();

                if (intakeRetract) {
                    robot.intake.deploy();

                    long starTime = startTime;
                    /*if (System.currentTimeMillis() - startTime < 2) {

                        robot.intake.reverse();
                    }
                    else{
                        robot.intake.stop();
                    }*/
            /*        robot.intake.reverse();
                    sleep(2);
                    robot.intake.stop();

                }

                intakeRetract = false;
                intakeOnForward = false;
                // }
            }*/
            if (robot.v4bArm.armShift==false && robot.v4bArm.armMove==false){

                //if ( robot.v4bArm.armMotor1.getEncoderValue() > last ){

                if ( robot.v4bArm.armMotor1.getEncoderValue() > last){


                    //&& (robot.v4bArm.armMotor1.getEncoderValue() < last-10)
                    // double difference = last-robot.v4bArm.armMotor1.getEncoderValue();


                    // robot.v4bArm.retract(0.8f);

                    robot.v4bArm.reverse(1000);


                }
                else {

                    robot.v4bArm.stop();

                }

            }


            /*if (gamepad1.right_bumper && intakeRetract) {

                robot.intake.deploy();

                long starTime = startTime;



                robot.intake.reverse();

                sleep(2000);

               /* if(System.currentTimeMillis()-starTime> 2000){


                    robot.intake.reverse();

                }*/


              /*  robot.intake.stop();


            }

            intakeRetract = gamepad1.right_bumper;*/



           if (robot.v4bArm.armMotor1.getEncoderValue() < 0){

                        robot.v4bArm.resetAllEncoders();

                    }

            //intake Control





          /*  if (gamepad2.a && !previousArmToggle){



           //     robot.v4bArm.resetAllEncoders();


                if (armOn == 0){
                    if (robot.v4bArm.armMotor1.getEncoderValue() < 300) {
                        //robot.v4bArm.work(0.5f,100);
                        robot.v4bArm.start(300);

                        telemetry.addData("message", "NOT THERE YET");


                  ;


                    } else {

                        telemetry.addData("message", "THERE NOW");

                        robot.v4bArm.stop();

                        armOn = 1;


                    }

                }
                else if (armOn == 1 && gamepad2.a){

                    if (robot.v4bArm.armMotor1.getEncoderValue() < 500) {
                        //robot.v4bArm.work(0.5f,100);
                        robot.v4bArm.start(550);

                        telemetry.addData("message", "NOT THERE YET");


                    } else {

                        telemetry.addData("message", "THERE NOW");

                        robot.v4bArm.stop();
                        armOn = 2;


                    }





                }
                else if (armOn ==2 && gamepad2.a){
                    if (robot.v4bArm.armMotor1.getEncoderValue() < 600) {
                        //robot.v4bArm.work(0.5f,100);
                        robot.v4bArm.start(600);

                        telemetry.addData("message", "NOT THERE YET");


                    } else {

                        telemetry.addData("message", "THERE NOW");

                        robot.v4bArm.stop();
                        armOn = 0;


                    }
                }
            }
            previousArmToggle = gamepad2.a;*/




            // Set drive power
            robot.setDrivePower(-x, y, rx);




           /*  telemetry.addData("xVel", robot.robotPose.getXVelocity());
             telemetry.addData("yVel", robot.robotPose.getYVelocity());
             telemetry.addData("xAccel", robot.robotPose.getXAcceleration());
             telemetry.addData("yAccel", robot.robotPose.getYAcceleration());

             telemetry.addData("rot", robot.robotPose.getHeading());
             telemetry.addData("x", robot.robotPose.getX());
             telemetry.addData("y", robot.robotPose.getY());


             telemetry.addData("Middle Odometer", (double) ((Motor) robot.components[2]).getEncoderValue());
             telemetry.addData("Left Odometer", (double) ((Motor) robot.components[0]).getEncoderValue());
             telemetry.addData("Right Odometer", -(double) ((Motor) robot.components[1]).getEncoderValue());

             telemetry.addData("Flywheel 1 RPM", robot.flywheel.currentRPM);
             telemetry.addData("Flywheel 2 RPM", robot.flywheel.currentRPM2);

              telemetry.addData("Shooter 1 ", robot.flywheel.shooter1Speed);
              telemetry.addData("Shooter 2", robot.flywheel.shooter2Speed);

              telemetry.addData("toggle", robot.previousPrimeShooter);* */

        }
    }

    // Simple custom robot class
    // Holds the hardware for a basic mecanum drive

}