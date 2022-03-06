package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public Component[] components;
    public Mecanum drivetrain;
    public Gyro gyro;
    public FlyWheel flywheel;
    public Intake intake;
   // public WobbleGoal wobbleClaw;
    public Mag mag;
    public Flicker flicker;


    public boolean previousMotorToggle = false;
    public boolean previousCarouselToggle = false;
    public boolean previousServoToggle = false;
    public boolean previousWGClaw = false;
    public boolean previousWGArm = false;

    public Carousel carousel;
    public Depositor depositor;
    public v4bArm v4bArm;





    /*
    Odometer 1 = R
    Odometer 2 = L
    Odometer 3 = M
     */

    /*public Pose2d robotPose = new Pose2d(
            15.3543307,
            0.1673228,
            0.6968503935
    );*/


    public Pose robotPose = new Pose(
            Math.PI/2, Math.PI/2, 0.0,
            7.41830709, -7.41830709, 0.5748031,
            0.5748031, 0.5748031, 3.75,
            0.6968503935
            );

    //Autonomous Constants
    public float currentR = 0.0f;
    public float targetR = 0.0f;

    public float currentY = 0.0f;
    public float targetY = 0.0f;
    public float highestY = 0.0f;

    public float currentX = 0.0f;
    public float targetX = 0.0f;
    public float highestX = 0.0f;

    public float correctionX = 0.0f;
    public float correctionY = 0.0f;
    public float correctionR = 0.0f;

    public boolean pid = true;

    private final float rKPR = 0.012f;
    private final float rKIR = 0.000005f;
    private final float rKDR = 0.0000f;

    private final float yBASE = 0.5f;
    private final float xBASE = 0.06f;

    private final float yKPR = 0.06f;
    private final float yKIR = 0.000002f;
    private final float yKDR = 0.00f;

    private final float xKPR = 0.17f;
    private final float xKIR = 0.000007f;
    private final float xKDR = 0.00f;

    private float lastX = 0;
    private float lastY = 0;

    public float xCor = 0;
    public float yCor = 0;
    public float rCor = 0;

    public boolean auton = false;

    public double microstepWobble = 0;

    public boolean intakeRunning = false;


    private PIDController pidYDistance = new PIDController(0f, yKPR, yKIR, yKDR, false);
    private PIDController pidXDistance = new PIDController(0f, xKPR, xKIR, xKDR, false);
    private PIDController pidRotation = new PIDController(0.0f, rKPR, rKIR, rKDR, true);


    public SampleMecanumDrive drive;
    public boolean previousPrimeShooter = false;
    public boolean previousDepToggle = false;
    public boolean previousArmToggle = false;
    public boolean previousArmToggle2 = false;
    public boolean previousArmToggle3 = false;
    public boolean armEnabled= false;
    public boolean shooterPrimed = false;
    private boolean wobbleGoalClawOpen = false;
    private boolean wobbleGoalArmOpen = false;
    private int carouselOn = 0;
    private boolean intakeOn = false;
    private boolean depositorOn = false;

    private int depOn = 0;
    public int armOn = 0;
    private boolean shouldLower = true;
    private boolean intakeOnForward = false;
    private boolean intakeOnReverse = false;

    private boolean armPrimer = false;

    private boolean armGo1 = false;
    private boolean armGo2 = false;
    private boolean armGo3 = false;


    ///public Pose2d startPose = new Pose2d(0, 0, 0);




    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

        this.components = new Component[]{
                new Motor(3, "backLeft", map, true),                            //0 left odometer
                new Motor(2, "backRight", map, false),                          //1 right odometer
                new Motor(1, "frontLeft", map, true),                           //2 middle odometer
                new Motor(0, "frontRight", map, false),                         //3
                new Motor(1, "carouselSpinner", map, true),                     //4
                new Motor(2, "leftArm", map, true),                             //5
                new Motor(3, "rightArm", map, false),                           //6
                new Motor(0, "intakeMotor", map, false),                       //7
                                      //8
                new StepperServo(0, "intakeServo", map),                          //8
                                   //10
                new StepperServo(1, "depositArm",map)                           //9
                                    //12


        };




       // this.flywheel = new FlyWheel(components[4], components[5]);


        this.intake = new Intake((StepperServo) components[8], (Motor) components[7]);

        this.carousel = new Carousel(components[4]);
        this.depositor = new Depositor(components[9]);
        this.v4bArm = new v4bArm(components[5], components[6]);





    }

    public void updateLoop(){



      //  flywheel.updateRPM();
      /*  v4bArm.updateDistance();*/
        carousel.updateRPM();
       v4bArm.updateDistance();




    }




    /* -- Subsystem Control -- */
    public void toggleIntake(boolean a) {
        //intake Control


       if (a && !previousMotorToggle){
            if (intakeOn){
                intake.deploy();
                intakeOn = false;
            } else {
                intake.retract();
                intakeOn = true;
            }
        }
        previousMotorToggle = a;


    }

    public void toggleCarousel(boolean x) {

        if (x && !previousCarouselToggle){
            if (carouselOn == 0){
                carousel.run(-0.4f);
                carouselOn = 1;
            }
            else if (carouselOn == 1 && x){

                carousel.turbo(-0.7f);
                carouselOn = 2;


            }
            else if (carouselOn ==2 && x){
                carousel.shut();
                carouselOn = 0;
            }
        }
        previousCarouselToggle = x;
        //intake Control


    }

    public void toggleArm(boolean x) {

        if (x && !previousCarouselToggle){
            if (carouselOn == 0){
                carousel.run(-0.2f);
                carouselOn = 1;
            }
            else if (carouselOn == 1 && x){

                carousel.turbo(-0.7f);
                carouselOn = 2;


            }
            else if (carouselOn ==2 && x){
                carousel.shut();
                carouselOn = 0;
            }
        }
        previousCarouselToggle = x;
        //intake Control


    }

    public void toggleRedCarousel(boolean x) {

        if (x && !previousCarouselToggle){
            if (carouselOn == 0){
                carousel.run(0.4f);
                carouselOn = 1;
            }
            else if (carouselOn == 1 && x){

                carousel.turbo(0.4f);
                carouselOn = 2;


            }
            else if (carouselOn ==2 && x){
                carousel.shut();
                carouselOn = 0;
            }
        }
        previousCarouselToggle = x;
        //intake Control


    }

    public void deployIntake(boolean val){

        intake.deploy();

        intake.retract();


    }

    public void intakeOn(float val) {
        if (val >= 0.5f){
            intake.start();
            intakeOnForward = true;
        } else if (!intakeOnReverse){
            intake.stop();
            intakeOnForward = false;
        }
    }

    public void intakeReverse(float val) {
        if (val >= 0.5f){
            intake.reverse();
            intakeOnReverse = true;
        } else if (!intakeOnForward){
            intake.stop();
            intakeOnReverse = false;
        }
    }

    public void carouselOn(float val) {



        //convert val to RPM
        if (val >= 0.5f){
            carousel.start(5000);

        }

        else{
            carousel.stop();

        }
    }


    public void armOn(boolean b) {

        if (b && !previousArmToggle){
            if (armEnabled){


                while( v4bArm.armMotor1.getEncoderValue() < 0.5){

                     v4bArm.run(0.4f);

                }

                armEnabled = false;
            } else {

                while( v4bArm.armMotor1.getEncoderValue() > -0.5){

                    v4bArm.run(-0.4f);

                }
                armEnabled = true;
            }
        }
        previousArmToggle = b;


    }

    public void deposit(boolean b) {
        //Toggle claw open or close

        depositor.onClick(b);



    }

    public void toggleDeposit(boolean b) {
        //Toggle claw open or close

        if (b && !previousDepToggle){
            if (depositorOn){
                depositor.inTake();
                depositorOn = false;
            } else {
                depositor.outTake();
                depositorOn = true;
            }
        }
        previousDepToggle = b;


    }

    public void toggleDeposit2(boolean b){

        if (b && !previousDepToggle){
            if (depOn == 0){
                depositor.reset();
                depOn = 1;
            }
            else if (depOn == 1 && b){

                depositor.inTake();
                depOn = 2;


            }
            else if (depOn ==2 && b){
                depositor.outTake();
                depOn = 0;
            }
        }
        previousDepToggle = b;
    }

    public void moveLift(float speedDown, float speedUp){
        if(speedDown == 0 && speedUp == 0){
            v4bArm.brake();
        }else {
            if (speedDown != 0.0){
                v4bArm.retract(speedDown*0.75f);
            } else if (speedUp != 0.0){
                v4bArm.enable(speedUp*0.75f);
            }
        }
    }


    public void primeShooter(boolean x) {
        if (x && !previousPrimeShooter){
            if (shooterPrimed){
               // flywheel.stop();
                v4bArm.stop();
                shooterPrimed = false;
            } else {



                if (v4bArm.armMotor1.getEncoderValue() < 740) {

                    v4bArm.extend(740);

                } else {

                    shooterPrimed = true;

                }
                //flywheel.moveWheels();

            }
        }

        previousPrimeShooter = x;
    }

    public void armBottom(boolean b){



        if (b && previousArmToggle2) {



            armGo1 = true;




        }

        if(armGo1) {
            if (v4bArm.armMotor1.getEncoderValue() < 740) {

                v4bArm.extend(740);

            } else {

                v4bArm.stop();
                armGo1 = false;
            }

        }


        previousArmToggle2 = b;

    }

    public void armMiddle(boolean b){


        if (b && previousArmToggle3) {



            if (v4bArm.armMotor1.getEncoderValue() < 900) {

                v4bArm.extend(900);

            }

            else{

                v4bArm.stop();
            }



        }


        previousArmToggle3 = b;

    }

    public void armTop(boolean b){


        if (b && previousArmToggle) {



            if (v4bArm.armMotor1.getEncoderValue() < 1000) {

                    v4bArm.extend(1000);

                }

            else{

                v4bArm.stop();
            }



        }

        previousArmToggle = b;

    }

    public void armRetract(boolean b){


        if (b && previousArmToggle) {



            if (v4bArm.armMotor1.getEncoderValue() > 0) {

                v4bArm.reverse(1000);

            }

            else{

                v4bArm.stop();
            }



        }

        previousArmToggle = b;

    }

    public void armPreset(boolean b){


        if (b && !previousArmToggle){
           if (armOn == 0){
                if (v4bArm.armMotor1.getEncoderValue() < 755) {
                    //robot.v4bArm.work(0.5f,100);
                    v4bArm.start(750);



                }


               else {

                    v4bArm.stop();
                    armOn = 1;


                }


            }
            else if (armOn == 1 && b){

                if (v4bArm.armMotor1.getEncoderValue() <950) {
                    //robot.v4bArm.work(0.5f,100);
                    v4bArm.extend(30);



                }
                else {

                    v4bArm.stop();
                    armOn = 2;


                }
            }
            else if (armOn ==2 && b){
                if (v4bArm.armMotor1.getEncoderValue() < 1000) {
                    //robot.v4bArm.work(0.5f,100);
                    v4bArm.extend(30);



                }
                else {

                    v4bArm.stop();
                    armOn = 0;



                }
            }
            else{

                v4bArm.stop();
           }
        }
        previousArmToggle = b;
    }



    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = y - x + rx;
        double powerBackRight = y + x - rx;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }
        Motor backLeft = (Motor) components[0];
        Motor backRight = (Motor) components[1];
        Motor frontLeft = (Motor) components[2];
        Motor frontRight = (Motor) components[3];
        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed((float)powerBackLeft);
        backRight.setSpeed((float)powerBackRight);
    }

}
