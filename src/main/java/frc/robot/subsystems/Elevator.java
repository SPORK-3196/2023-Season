package frc.robot.subsystems;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
private static double kDt = 0.02;

private final Encoder m_encoder = new Encoder(1, 2);
private final MotorController m_MotorController = new PWMSparkMax(1);

private final TrapezoidProfile.Constraints m_Constraints =
 new TrapezoidProfile.Constraints(1.75, 0.75);
 private final ProfiledPIDController m_Controller = 
 new ProfiledPIDController(1.3, 0.0, 0.7, m_Constraints, kDt);
 
 public Elevator() {
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
 }
 //TODO find the controll method for the elevator
 public CommandBase ElevatorTop(){
    return this.runOnce(() -> m_MotorController.set(1));
 }
 public CommandBase ElevatorLow(){
    return this.runOnce(() -> m_MotorController.set(0));
 }
 public CommandBase elevatorSpecifyed(){
    return this.runOnce(() -> m_encoder.setDistancePerPulse(Constants.kElevatorDistance));
 }

 
}
