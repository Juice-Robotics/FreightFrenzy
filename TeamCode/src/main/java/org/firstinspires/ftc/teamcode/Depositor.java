package org.firstinspires.ftc.teamcode;

public class Depositor {
    public StepperServo depositor;


    // og values init 50, intake 25

    //50
    public int initialAngle = 65;


    //34

    //37
    public int intakeAngle = 45;

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