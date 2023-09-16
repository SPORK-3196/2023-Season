package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    public CANSparkMax shoulderMotor = 
        new CANSparkMax(ArmConstants.shoulderPort,MotorType.kBrushless);
    
    public CANSparkMax elbowMotor = 
        new CANSparkMax(ArmConstants.elbowPort,MotorType.kBrushless);


   
    public RelativeEncoder elbowEncoder = elbowMotor.getEncoder();
    public RelativeEncoder shoulderEncoder = shoulderMotor.getEncoder();


    public SparkMaxLimitSwitch elbowSwitchBottom = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    public SparkMaxLimitSwitch elbowLimitSwitch = elbowMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    public SparkMaxLimitSwitch shoulderLimitSwitchTop = elbowMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
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
        
        elbowKP = 1;
        elbowKD = 0;
        elbowKI = 0;
        elbowFFa = 0;
        elbowMax = 0;//0.3;
        elbowMin = -0.2;//-0.3;
        

        shoulderFFa = 0;
        shoulderFFb = 0;
        shoulderMax = 0.1;
        shoulderMin = -0.1;
        shoulderKP = 0.5;
        shoulderKD = 0;
        shoulderKI = 0;

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
        elbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    }

    public void turnShoulderOff(){
        shoulderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        shoulderMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    }
    public double getShoulderTick(){
        return  shoulderEncoder.getPosition();
    }
    public double getElbowTick(){
        return  elbowEncoder.getPosition(); 

    }
    public boolean isResetElbow(){
        System.out.println("");
        return elbowLimitSwitch.isPressed();
    }
    public boolean isStopElbow(){
        return elbowSwitchBottom.isPressed();
    }
    public boolean isStopShoulder(){
        return shoulderLimitSwitchTop.isPressed();
    }
    public boolean isResetShoulder(){
        System.out.println("");
        return shoulderLimitSwitch.isPressed();
    }

    public double getElbowAngle(){
        return ((elbowEncoder.getPosition() / (20*2)) * 360) + 165 + getShoulderAngle();
    }

    public double getShoulderAngle(){
        return -((shoulderEncoder.getPosition() / (36*2)) * 360) - 75;
    }
}

