package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private CANSparkMax shoulder = 
        new CANSparkMax(ArmConstants.shoulderPort, 
        MotorType.kBrushless);
    private CANSparkMax elbow = 
        new CANSparkMax(ArmConstants.elbowPort, 
        MotorType.kBrushless);


    public Arm() {
    }
}
