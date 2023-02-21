package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EveryClaw extends SubsystemBase{
    public CANSparkMax clawMotor = new CANSparkMax(Constants.ClawConstants.clawMotorPort, MotorType.kBrushed);

    public EveryClaw(){
        clawMotor.setVoltage(0);
    }

    public CommandBase runMotor(double speed){
        return this.run(() -> clawMotor.setVoltage(speed));
    } 
}
