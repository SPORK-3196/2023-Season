package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TurretConstants;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Turret extends SubsystemBase {
    //Initialize motor
    private CANSparkMax turret = 
        new CANSparkMax(TurretConstants.turretPort, MotorType.kBrushless );


    public void rotateXDegrees(double x){
        
        //turn degrees into time to rotate x degrees
        turret.set(TurretConstants.idealSpeed); 
        //wait time
        turret.stopMotor();
    }

    public void rotate90Degrees(){
        turret.setVoltage(TurretConstants.motorVolts);
        //wait set time
    }

    public void disableTurret(){
        turret.disable();
    }
}

