package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    public CANSparkMax shoulderMotor = 
        new CANSparkMax(ArmConstants.shoulderPort, MotorType.kBrushless);
    
    public CANSparkMax elbowMotor = 
        new CANSparkMax(ArmConstants.elbowPort,MotorType.kBrushless);

    public RelativeEncoder elbowEncoder = elbowMotor.getEncoder();
    public RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();

    public SparkMaxPIDController elbowController;
    public SparkMaxPIDController shoulderController;

    public ArmFeedforward shoulderFeedforward = new ArmFeedforward(
        Constants.ArmConstants.shoulderKsVolts, 
        Constants.ArmConstants.shoulderKgVolts, 
        Constants.ArmConstants.shoulderKvVoltSecondPerRad, 
        Constants.ArmConstants.shoulderKaVoltSecondSquaredPerRad);
    
    public ArmFeedforward elbowFeedForward = new ArmFeedforward(
        Constants.ArmConstants.elbowKsVolts, 
        Constants.ArmConstants.elbowKgVolts, 
        Constants.ArmConstants.elbowKvVoltSecondPerRad, 
        Constants.ArmConstants.elbowKaVoltSecondSquaredPerRad);

    private double elbowKI, elbowKD, elbowKP, shoulderKI, shoulderKD, shoulderKP;
    public Arm(){
        elbowController = elbowMotor.getPIDController();
        shoulderController = shoulderMotor.getPIDController();

        elbowKP = .007;
        elbowKD = 0;
        elbowKI = 0;
        shoulderKP = .008;
        shoulderKD = 0;
        shoulderKI = 0;

        elbowController.setP(elbowKP);
        elbowController.setD(elbowKD);
        elbowController.setI(elbowKI);
        shoulderController.setP(shoulderKP);
        shoulderController.setD(shoulderKD);
        shoulderController.setI(shoulderKI);


    }
    public CommandBase runElbowMotor(double speed){
        return this.run(() -> elbowMotor.setVoltage(speed));
    }

    public CommandBase runShoulderMotor(double speed){
        return this.run(() -> shoulderMotor.setVoltage(speed));
    }

    public CommandBase turnElbowOff(){
        return this.runOnce(() -> elbowMotor.stopMotor());
    }

    public CommandBase turnShoulderOff(){
        return this.runOnce(() -> shoulderMotor.stopMotor());
    }
}
