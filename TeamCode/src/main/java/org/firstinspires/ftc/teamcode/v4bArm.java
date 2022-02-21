package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class v4bArm{
    public Motor armMotor1;
    public Motor armMotor2;
    private long lastTime;

    ;

    public static double targetDistance = 1000;

    public double lastTarget = 5300;

    final double KP = 0.0005, KI = 0.00, KD = 0.00;

    public static PIDController armPID;
    public static PIDController armPID2;

    public double armMotor1Distance = 0.0;
    public double armMotor2Distance = 0.0;

    private double armMotorFF = 0.7;
    public double currentDistance = 0;
    public double currentDistance2 = 0;



    boolean armShift = false;


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
            armMotor1.setSpeed((float) Range.clip(armMotor1Distance, -1, 1));
            armMotor2.setSpeed((float) Range.clip(armMotor1Distance, -1, 1));
        } else {
            armMotor1.setSpeed(0);
            armMotor2.setSpeed(0);
        }



    }

    public void move(double distance){


          armShift = true;



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
        armMotor1.setSpeed(speed);
        armMotor2.setSpeed(speed);


    }

    public void brake(){
        armMotor1.setSpeed(0);
        armMotor2.setSpeed(0);


    }

    public void extend(double distance){
        armMotor1.setSpeed(0.9f);
        armMotor2.setSpeed(0.9f);

    }

    public void retract(float speed){
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

        armMotor1.setSpeed(0);
        armMotor2.setSpeed(0);



    }
}

