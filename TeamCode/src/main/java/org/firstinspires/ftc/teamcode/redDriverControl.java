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
public class redDriverControl extends LinearOpMode {


    boolean changed = false;

    public boolean previousArmToggle = false;
    public boolean armEnabled= false;

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

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
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
       //     robot.intakeOn(gamepad1.right_trigger);

            robot.intakeReverse(gamepad1.left_trigger);

            //   robot.armOn(gamepad1.b);

            //  robot.toggleDeposit(gamepad1.b);
            robot.toggleDeposit2(gamepad2.y);
            robot.toggleRedCarousel(gamepad1.x);

            robot.armPreset(gamepad2.b);

            robot.moveLift(gamepad2.left_trigger, gamepad2.right_trigger);





         /*   if (gamepad1.b && !previousArmToggle){
                if (armEnabled){

                    while( robot.v4bArm.armMotor1.getEncoderValue() > -0.2){

                        robot.v4bArm.run(-0.3f);


                    }

                    armEnabled = false;
                } else {

                    while( robot.v4bArm.armMotor1.getEncoderValue() < 0.2){

                        robot.v4bArm.run(0.3f);

                    }
                    armEnabled = true;
                }
            }
            previousArmToggle = gamepad1.b;*/









            /*if(gamepad1.a && !changed) {
                if(robot.intake.intakeServo1.getAngle() == 0){
                    robot.intake.deploy();

                }
                else {robot.intake.retract();}
                changed = true;
            } else if(!gamepad1.a) {
                changed = false;
            }*/

           /* robot.carouselOn(gamepad2.left_stick_y);


            robot.armOn(gamepad2.right_stick_x);

            robot.deposit(gamepad1.a);*/



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
