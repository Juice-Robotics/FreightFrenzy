package.org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

public class arm {
    public Motor armMotor1;
    public Motor armMotor2;
    private long lastTime;
    
    private double pastRPM;
    private double pastRPM2;

    public double targetDistance = 5300;

    public double lastTarget = 5300;

    final double KP = 0.0005, KI = 0.00, KD = 0.00;

    PIDController armPID = new PIDController(targetDistance, KP, KI, KD, false);
    PIDController armPID2 = new PIDController(targetDistance, KP, KI, KD, false);

    public double armMotor1Distance = 0.0;
    public double armMotor2Distance = 0.0;
    
    private double armMotorFF = 0.7;
    public double currentDistance = 0;
    public double currentDistance2 = 0;

    public arm(Component armMotor1, Component armMotor2){
        this.armMotor1 = (Motor) armMotor1;
        this.armMotor2 = (Motor) armMotor2;

        armMotor1Distance = 1;
        armMotor2Distance = 1;
        
        lastTime = System.currentTimeMillis();
        
        pastRPM = this.armMotor1.getEncoderValue();
        pastRPM2 = this.armMotor2.getEncoderValue();
    }

    public void updateDistance(){

        if (lastTarget != targetDistance){
            armPID = new PIDController(targetDistance, KP, KI, KD, false);
            armPID2 = new PIDController(targetDistance, KP, KI, KD, false);
        }

        long time = System.currentTimeMillis();
        double currentRPM = armMotor1.getEncoderValue();
        double currentRPM2 = armMotor2.getEncoderValue();
        lastTarget = targetDistance;

        currentDistance = ((armMotor1.getEncoderValue() - pastRPM) * (time - lastTime)) * ((60000 * 40)/(28*22));
        currentDistance2 = ((armMotor2.getEncoderValue() - pastRPM2) * (time - lastTime)) * ((60000 * 40)/(28*22));

        lastTime = time;

        pastRPM = currentRPM;
        pastRPM2 = currentRPM2;

        double correction1 = armPID.update(currentDistance);
        double correction2 = armPID2.update(currentDistance2);

        armMotor1Distance = correction1 + armMotorFF;
        armMotor2Distance = correction2 + armMotorFF;

        if (armShift){
            armMotor1.setDistance((float) Range.clip(armMotor1Distance, -1, 1));
            armMotor2.setDistance((float) Range.clip(armMotor1Distance, -1, 1));
        } else {
            armMotor1.setDistance(0);
            armMotor2.setDistance(0);
        }
    

public arm (Motor armMotor1, Motor armMotor2)    {
    this.armMotor1 = armMotor1;
    this.armMotor2 = armMotor2;
}
public void start (){
    armMotor1.setSpeed(0.4f);
    armMotor2.setSpeed(0.4f);
}
public void reverse (){
    armMotor1.setSpeed(-0.4f);
    armMotor2.setSpeed(-0.4f);
}
public void resetMotorSpeeds (){
    armMotor1 = 0.7;
    armMotor2 = 0.7;
}
public void stop ();
    armMotor1.setSpeed(0);
    armMotor2.setSpeed(0);
}