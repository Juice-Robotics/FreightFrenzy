package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class carousel {

    public Motor carousel;
    double targetRPM = 30;

    public Carousel(Component carouselMotor){
        
        this.carousel = (Motor) carouselMotor; 
    }
    
    final double KP = 0.0001;
    final double KI = 0;
    final double KD = 0;
    
    public double currentRPM = 0;
    
    boolean spinmotor = false;

    PIDController spinmotorPID = new spinmotor (targetRPM, KP, KI, KD, false);

    public void start(){

        spinmotor = true;
    }
   

    public void updateRPM(){

        if (targetRPM != currentRPM) {
            spinmotorPID = new PIDController(targetRPM, KP, KI, KD, false);
        }
    
        double correction = spinmotorPID.update(currentRPM);
        
    }

    public void stop(){
        spinmotor = false;
    }
}



