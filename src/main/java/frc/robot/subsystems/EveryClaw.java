package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EveryClaw extends SubsystemBase{
    public CANSparkMax clawMotor = new CANSparkMax(Constants.ClawConstants.clawMotorPort, MotorType.kBrushed);

    public EveryClaw(){
        clawMotor.set(0);
    }

    public void runMotor(double speed){
        clawMotor.set(speed);
    } 
    public void stopMotor(){
        clawMotor.stopMotor();
    }
}


