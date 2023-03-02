package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    public CANSparkMax turretMotor = new CANSparkMax(10, MotorType.kBrushless);
    public SparkMaxPIDController turretController = turretMotor.getPIDController();
    public RelativeEncoder turretEncoder = turretMotor.getEncoder();
    public double kp, ki, kd, max;
    
    public Turret(){
        this.resetTurretEncoder();
        kp = .008;
        ki = 0;
        kd = 0;
        turretController.setP(kp);
        turretController.setD(kd);
        turretController.setI(ki);
        max = 1;
        turretController.setOutputRange(kd, max);
    }

    public void resetTurretEncoder(){
    }
    
    public void moveMotor(double speed){
        turretMotor.set(speed);

    }

    public void setPIDSetpoint(double degrees){
        turretController.setReference(degrees * Constants.TurretConstants.motorRotationsPerDegree, CANSparkMax.ControlType.kPosition);
    }
}