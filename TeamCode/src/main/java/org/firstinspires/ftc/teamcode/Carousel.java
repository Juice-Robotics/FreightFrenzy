package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;

public class Carousel {

    public Motor carousel;
    public static double targetRPM;


    final double KP = 0.0005, KI = 0.00, KD = 0.00;
    
    public static double currentRPM = 0;

    private long lastTime;

    private double pastTicks;

    public double lastTarget = 5300;
    
    boolean spinmotor = false;

    double carouselSpeed;

    private double shooterFF = 0.7;

    public double spinTime;
    public static PIDController spinmotorPID;

    public Carousel(Component carouselMotor){

        carousel = (Motor) carouselMotor;
        //this.targetRPM  = speed;



        carouselSpeed = 1;


        lastTime = System.currentTimeMillis();

        pastTicks = carousel.getEncoderValue();

    }

    public void start(double rpm){

        spinmotor = true;
        targetRPM = rpm;
        spinmotorPID = new PIDController (targetRPM, KP, KI, KD, false);
       // FtcDashboard dashboard = FtcDashboard.getInstance();
       // dashboard.addConfigVariable("PIDController", "KP", KP);


    }


   

    public void updateRPM(){

        if (lastTarget != targetRPM){
            spinmotorPID = new PIDController(targetRPM, KP, KI, KD, false);

        }

        long time = System.currentTimeMillis();
        double currentTicks = carousel.getEncoderValue();

        lastTarget = targetRPM;

        currentRPM = ((carousel.getEncoderValue() - pastTicks) / (time - lastTime)) * ((60000 * 40)/(28*22));


        lastTime = time;

        pastTicks = currentTicks;


        double correction1 = spinmotorPID.update(currentRPM);


        carouselSpeed= correction1 + shooterFF;


        if (spinmotor){
            carousel.setSpeed((float) Range.clip(carouselSpeed, -1, 1));

        } else {
            carousel.setSpeed(0);

        }
        
    }


    public void reverse () {
        carousel.setSpeed(-0.4f);

    }

    public void run(float speed){

        carousel.setSpeed(speed);
    }

    public void turbo(float speed){
        carousel.setSpeed(speed);


    }
    public void resetMotorSpeeds(){
        carouselSpeed= 0.7;

    }

    public void resetAllEncoders(){
        carousel.resetEncoder();

    }

    public void shut(){

        carousel.setSpeed(0);
    }

    public void stop(){

        spinmotor = false;
        shut();
    }
}



