package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EveryClaw extends SubsystemBase{
    public CANSparkMax clawMotorleft = new CANSparkMax(Constants.ClawConstants.LeftclawMotorPort, MotorType.kBrushless);
    public CANSparkMax clawMotorRight = new CANSparkMax(Constants.ClawConstants.RightClawMotorPort, MotorType.kBrushless);


    public EveryClaw(){
        clawMotorleft.set(-0.03);
        clawMotorRight.set(0.03);
    }

    public void runMotor(double speed){
        clawMotorleft.set(speed);
        clawMotorRight.set(-speed);
    } 
    public void stopMotor(){
        clawMotorleft.stopMotor();
        clawMotorRight.stopMotor();
    }
}


