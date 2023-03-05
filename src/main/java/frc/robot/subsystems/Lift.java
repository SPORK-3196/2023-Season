package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class Lift extends SubsystemBase {

    public CANSparkMax liftMotor = new CANSparkMax(Constants.LiftConstants.liftMotorID, MotorType.kBrushless);
    public RelativeEncoder liftEncoder = liftMotor.getEncoder();

    public SparkMaxLimitSwitch lifSwitch = liftMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

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

    public void runLift(double speed){
      liftMotor.set(speed);
    }

    public void stopLift(){
      liftMotor.stopMotor();
    }

    public double getEncoderTick(){
      return liftEncoder.getCountsPerRevolution() * liftEncoder.getPosition(); 
    }

    public boolean isResetLift(){
      return lifSwitch.isPressed();
    }

}