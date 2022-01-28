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
    public SampleMecanumDrive rrdrive;

    public boolean previousMotorToggle = false;
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
    public boolean shooterPrimed = false;
    private boolean wobbleGoalClawOpen = false;
    private boolean wobbleGoalArmOpen = false;
    private boolean intakeOn = false;
    private boolean shouldLower = true;
    private boolean intakeOnForward = false;
    private boolean intakeOnReverse = false;

    ///public Pose2d startPose = new Pose2d(0, 0, 0);




    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new SampleMecanumDrive(map);

        this.components = new Component[]{
                new Motor(3, "backLeft", map, true),                            //0 left odometer
                new Motor(2, "backRight", map, false),                          //1 right odometer
                new Motor(1, "frontLeft", map, true),                           //2 middle odometer
                new Motor(0, "frontRight", map, false),                         //3
                new Motor(2, "carouselSpinner", map, true),                     //4
                new Motor(3, "leftArm", map, true),                             //5
                new Motor(0, "rightArm", map, false),                           //6
                new Motor(0, "intakeMotor1", map, false),                       //7
                new Motor(0, "intakeMotor1", map, false),                       //8
                new StepperServo(0, "intakeServoFront", map),                          //9
                new StepperServo(0, "intakeServoRear", map),                           //10
                new StepperServo(0, "depositArm",map),                                 //11
                new StepperServo(0,"depositArm2", map),                                //12


        };




        this.flywheel = new FlyWheel(components[4], components[5]);


        this.intake = new Intake((StepperServo) components[11], (StepperServo) components[12], (Motor) components[9], (Motor) components[10]);

        this.carousel = new Carousel(components[14]);
        this.depositor = new Depositor(components[15]);
        this.v4bArm = new v4bArm(components[16], components[17]);





    }

    public void updateLoop(){



      //  flywheel.updateRPM();
        v4bArm.updateDistance();
        carousel.updateRPM();




    }




    /* -- Subsystem Control -- */
    public void toggleIntake(boolean a) {
        //intake Control
        intake.deploy();

        if (a && !previousMotorToggle){
            if (intakeOn){
                intake.stop();
                intakeOn = false;
            } else {
                intake.start();
                intakeOn = true;
            }
        }
        previousMotorToggle = a;
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
        if (val >= 0.5f){
            carousel.start(20);

        } else if (!intakeOnReverse){
            carousel.stop();

        }
    }


    public void armOn(float val) {

        v4bArm.armMotor1.setSpeed(val);

        v4bArm.armMotor2.setSpeed(val);
    }

    public void deposit(boolean b) {
        //Toggle claw open or close

        depositor.onClick(b);
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
