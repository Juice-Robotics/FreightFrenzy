package org.firstinspires.ftc.teamcode;

public class Depositor {
    public StepperServo depositor;

    public int initialAngle = 0;            /* arbitrary values for now */
    public int desiredAngle = 45;

    public Depositor(Component depositorMotor){
        this.depositor = (StepperServo) depositorMotor;

        depositor.setAngle(initialAngle);
    }

    public void onClick(boolean click){
        if(click){
            depositor.setAngle(desiredAngle);
        } else {
            depositor.setAngle(initialAngle);
        }
    }

    public void start(){


    }

    public void stop(){


    }

    public void reverse(){

    }




}