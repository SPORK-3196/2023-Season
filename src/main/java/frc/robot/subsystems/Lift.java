package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Lift extends SubsystemBase {

    public CANSparkMax liftMotor = new CANSparkMax(Constants.LiftConstants.liftMotorID, MotorType.kBrushless);
    public RelativeEncoder liftEncoder = liftMotor.getEncoder();

    public SparkMaxPIDController liftController;

    double kp, kd, ki;

    public Lift(){
      liftController = liftMotor.getPIDController();

      kp = Constants.LiftConstants.liftKP;
      kd = Constants.LiftConstants.liftKD;
      ki = Constants.LiftConstants.liftKI;

      liftController.setP(kp);
      liftController.setD(kd);
      liftController.setI(ki);
    }

    public RelativeEncoder getEncoder(){
      return liftEncoder;
    }

    public CommandBase runLift(double speed){
      return this.run(() -> liftMotor.setVoltage(speed));
    }

    public CommandBase stopLift(){
      return this.runOnce(() -> liftMotor.stopMotor());
    }
}