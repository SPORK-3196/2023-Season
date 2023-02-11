package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    public double ElbowEnoder= 0;
    public double ShoulderEncoder= 0;

    
    /*public CANSparkMax ShoulderMotor = 
        new CANSparkMax(ArmConstants.shoulderPort, MotorType.kBrushless);
    
    public CANSparkMax ElbowMotor = 
        new CANSparkMax(ArmConstants.elbowPort,MotorType.kBrushless);*/

    public CANSparkMax ShoulderMotor = new CANSparkMax(ArmConstants.shoulderPort, MotorType.kBrushless);
    public SparkMaxPIDController ShoulderPID = ShoulderMotor.getPIDController();

    public CANSparkMax ElbowMotor = new CANSparkMax(9, MotorType.kBrushless);
    public SparkMaxPIDController ElbowPID = ElbowMotor.getPIDController();
    public Object shoulderOut;



    
    public void DefaltCommand(){
    //TODO run arm commands 
       // setDefaultCommand(new RunArm());
    }
}
