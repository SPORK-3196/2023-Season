package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
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

    public SparkMaxLimitSwitch elbowLimitSwitch = elbowMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    public SparkMaxLimitSwitch shoulderLimitSwitch = shoulderMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);


    public SparkMaxPIDController elbowController;
    public SparkMaxPIDController shoulderController;

    public ArmFeedforward shoulderFeedforward = new ArmFeedforward(
        Constants.ArmConstants.shoulderKsVolts, 
        Constants.ArmConstants.shoulderKgVolts, 
        Constants.ArmConstants.shoulderKvVoltSecondPerRad, 
        Constants.ArmConstants.shoulderKaVoltSecondSquaredPerRad);
    /* 
    public ArmFeedforward elbowFeedForward = new ArmFeedforward(
        Constants.ArmConstants.elbowKsVolts, 
        Constants.ArmConstants.elbowKgVolts, 
        Constants.ArmConstants.elbowKvVoltSecondPerRad, 
        Constants.ArmConstants.elbowKaVoltSecondSquaredPerRad);
*/
    public double elbowKI, elbowKD, elbowKP, shoulderKI, shoulderKD, shoulderKP, elbowFFa, shoulderFFa, shoulderFFb, elbowMax, elbowMin, shoulderMax, shoulderMin;
    public Arm(){
        elbowController = elbowMotor.getPIDController();
        shoulderController = shoulderMotor.getPIDController();

        //TODO change these values
        
        elbowKP = 1.8;
        elbowKD = 0;
        elbowKI = .0006;
        elbowFFa = 1.4;
        elbowMax = .13;
        elbowMin = -.11;
        

        shoulderFFa = -1.4;
        shoulderFFb = -1.4;
        shoulderMax = .13;
        shoulderMin = -.19;
        shoulderKP = 2;
        shoulderKD = 0;
        shoulderKI = .0007;

        elbowController.setP(elbowKP);
        elbowController.setD(elbowKD);
        elbowController.setI(elbowKI);
        elbowController.setOutputRange(elbowMin, elbowMax);
        elbowController.setIMaxAccum(3, 0);
        elbowController.setIZone(2.5);

        shoulderController.setP(shoulderKP);
        shoulderController.setD(shoulderKD);
        shoulderController.setI(shoulderKI);
        shoulderController.setOutputRange(shoulderMin, shoulderMax);
        shoulderController.setIMaxAccum(10, 0);
        shoulderController.setIZone(4.5);

    }
    public void runElbowMotor(double speed){
        elbowMotor.set(speed);
    }

    public void runShoulderMotor(double speed){
        shoulderMotor.set(speed);
    }

    public void turnElbowOff(){
        elbowMotor.stopMotor();
    }

    public void turnShoulderOff(){
        shoulderMotor.stopMotor();
    }
    public double getShoulderTick(){
        return  shoulderEncoder.getPosition();
    }
    public double getElbowTick(){
        return  elbowEncoder.getPosition(); 

    }
    public boolean isResetElbow(){
        return elbowLimitSwitch.isPressed();
    }

    public boolean isResetShoulder(){
        return shoulderLimitSwitch.isPressed();
    }

    public double getElbowAngle(){
        return ((elbowEncoder.getPosition() / (20*2)) * 360) + 165 + getShoulderAngle();
    }

    public double getShoulderAngle(){
        return -((shoulderEncoder.getPosition() / (36*2)) * 360) - 75;
    }
}

