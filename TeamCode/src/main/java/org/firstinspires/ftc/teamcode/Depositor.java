package org.firstinspires.ftc.teamcode;

public class Depositor {
    public StepperServo depositor;


    // og values init 50, intake 25

    public int initialAngle = 50;


    //34
    public int intakeAngle = 37;

    public int outakeAngle  = -35;

    /* arbitrary values for now */
    public int desiredAngle = 45;

    public Depositor(Component depositorMotor){
        this.depositor = (StepperServo) depositorMotor;


    }

    public void onClick(boolean click){
        if(click){
            depositor.setAngle(desiredAngle);
        } else {
            depositor.setAngle(initialAngle);
        }
    }

    public void reset() {

        depositor.setAngle(initialAngle);

    }

    public void inTake(){

        depositor.setAngle(intakeAngle);


    }
    public void outTake(){


        depositor.setAngle(outakeAngle);


    }

    public void reverse(){

    }




}