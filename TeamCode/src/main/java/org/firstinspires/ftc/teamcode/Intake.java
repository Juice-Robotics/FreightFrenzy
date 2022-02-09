package org.firstinspires.ftc.teamcode;

public class Intake {
    public StepperServo intakeServo1;
    public StepperServo intakeServo2;
    public Motor intakeMotor1;
    public Motor intakeMotor2;

    public Intake(StepperServo intakeServo1, Motor intakeMotor1){
        this.intakeServo1 = intakeServo1;

        this.intakeMotor1 = intakeMotor1;

    }

    public void deploy(){
        intakeServo1.setAngle(-30);

    }

    public void retract(){

        intakeServo1.setAngle(45);
    }

    public void start(){
        intakeMotor1.setSpeed(0.65f);

    }

    public void reverse(){
        intakeMotor1.setSpeed(-0.65f);

    }

    public void stop(){
        intakeMotor1.setSpeed(0);

    }

}
