package org.firstinspires.ftc.teamcode;

public class noodleIntake {
    public StepperServo intakeServo1;
    public StepperServo intakeServo2;
    public IntakeMotor intakeMotor1;
    public IntakeMotor intakeMotor2;
    public noodleIntake(){
       intakeMotor1 = new IntakeMotor(new Motor(1));
       intakeMotor2 = new IntakeMotor(new Motor(1));
       intakeServo1 = new StepperServo(new Servo(1));
       intakeServo2 = new StepperServo(new Servo(1));

    }

    public void deploy(){
        intakeServo1.setAngle(0);
        intakeServo2.setAngle(0);
    }

    public void sNoodle(){
    
        motor1.set(0.5);
        motor2.set(0.5);
    }
    
    public void reNoodle(){
    
        motor1.down(0.5);
        motor2.down(0.5);
    }

    public void stop(){
    
        motor1.set(0);
        motor2.set(0);
    }





}
