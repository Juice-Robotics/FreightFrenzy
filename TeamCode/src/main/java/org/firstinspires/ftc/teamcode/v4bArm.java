package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;


//@Config

public class v4bArm{
    public Motor armMotor1;
    public Motor armMotor2;
    private long lastTime;

    ;

    public static double targetDistance = 1000;

    public double lastTarget = 5300;

    public static double KP = 0.0005, KI = 0.00, KD = 0.00;

    public PIDController armPID;
    public  PIDController armPID2;

    public double armMotor1Distance = 0.0;
    public double armMotor2Distance = 0.0;


    private double K;
    private double targetAngle;
    private double armMotorFFg = Math.cos(targetAngle)*K;
    private double armMotorFF = 0.7;
    public double currentDistance = 0;
    public double currentDistance2 = 0;



    boolean armShift = false;
    boolean reverse = false;
    boolean armMove = false;
    boolean gravity = false;



    public v4bArm(Component armMotor1, Component armMotor2) {
        this.armMotor1 = (Motor) armMotor1;
        this.armMotor2 = (Motor) armMotor2;

        armMotor1Distance = 1;
        armMotor2Distance = 1;

        lastTime = System.currentTimeMillis();


    }

    public void updateDistance() {

        if (lastTarget != targetDistance) {
            armPID = new PIDController(targetDistance, KP, KI, KD, false);
            armPID2 = new PIDController(targetDistance, KP, KI, KD, false);
        }


        lastTarget = targetDistance;

        currentDistance = armMotor1.getEncoderValue();
        currentDistance2 = armMotor2.getEncoderValue();


        double correction1 = armPID.update(currentDistance);
        double correction2 = armPID2.update(currentDistance2);

        armMotor1Distance = correction1 + armMotorFF;
        armMotor2Distance = correction2 + armMotorFF;

        if (armShift) {
            armMotor1.setSpeed((float) Range.clip(armMotor1Distance, -1, 1)*0.45f);
            armMotor2.setSpeed((float) Range.clip(armMotor1Distance, -1, 1)*0.45f);

        }
        else if (reverse){
            armMotor1.setSpeed(((float) Range.clip(armMotor1Distance, -1, 1)*-1)*0.75f);
            armMotor2.setSpeed(((float) Range.clip(armMotor1Distance, -1, 1)*-1)*0.75f);

        }
      /*  else if (gravity){
            armMotor1.setSpeed(((float) Range.clip(armMotor1Distance, -1, 1)*-1)*0.60f);
            armMotor2.setSpeed(((float) Range.clip(armMotor1Distance, -1, 1)*-1)*0.60f);*/

        //}
        else {
            armMotor1.setSpeed(0);
            armMotor2.setSpeed(0);
        }



    }

    public void move(double distance){


          armShift = true;



    }

    public void reverse(double distance){


        reverse = true;

       // targetDistance = distance;



    }

    public void antiGravity(double distance){


        gravity = true;

        // targetDistance = distance;



    }


    /*public void moveback(double distance){


            while (armMotor1.getEncoderValue() > distance) {

                armMotor1.setSpeed(0.5f);
                armMotor2.setSpeed(0.5f);


            }




    }*/

    public void run (float speed){

        armMotor1.setSpeed(speed);
        armMotor2.setSpeed(speed);


    }
    public void start (double distance) {
        armShift = true;
        targetDistance = distance;


        PIDController armPID = new PIDController(targetDistance, KP, KI, KD, false);
        PIDController armPID2 = new PIDController(targetDistance, KP, KI, KD, false);


      //  FtcDashboard dashboard = FtcDashboard.getInstance();


    }

    public void work(float speed,int distance){

        armMotor1.setTarget(distance);
        armMotor2.setTarget(distance);
        armMotor1.setSpeed(speed);
        armMotor2.setSpeed(speed);

    }
     public void reverse () {
        armMotor1.setSpeed(-0.4f);
        armMotor2.setSpeed(-0.4f);

    }
    public void resetMotorSpeeds () {
        armMotor1.setSpeed(0.7f);
        armMotor2.setSpeed(0.7f);
    }

    public void enable(float speed){
        armMove = true;
        armMotor1.setSpeed(speed);
        armMotor2.setSpeed(speed);


    }

    public void brake(){
        armMove = false;
        armMotor1.setSpeed(0);
        armMotor2.setSpeed(0);


    }

    public void extend(double distance){


        armMove = true;
        armMotor1.setSpeed(1f);
        armMotor2.setSpeed(1f);

    }

    public void retract(float speed){

        armMove = true;
        armMotor1.setSpeed(speed*-1);
        armMotor2.setSpeed(speed*-1);
        //start(-distance);

    }

    public void resetAllEncoders(){
        armMotor1.resetEncoder();
        armMotor2.resetEncoder();

    }
    public void stop () {
        armShift =false;
        reverse = false;
       //gravity = false;
        armMove = false;

        armMotor1.setSpeed(0);
        armMotor2.setSpeed(0);



    }
}

