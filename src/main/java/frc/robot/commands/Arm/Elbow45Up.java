package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;


public class Elbow45Up extends ProfiledPIDSubsystem {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.elbowPort, MotorType.kBrushless);
  private final Encoder m_encoder =
      new Encoder(ArmConstants.elbowPort, ArmConstants.elbowPort);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.elbowKsVolts, ArmConstants.elbowKgVolts,
          ArmConstants.elbowKvVoltSecondPerRad, ArmConstants.elbowKaVoltSecondSquaredPerRad);

  public Elbow45Up() {
    super(
    new ProfiledPIDController(ArmConstants.kP,0,0,
    new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond,ArmConstants.kMaxAccelerationRadPerSecSquared)),0);
    m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
   
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
   
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
   
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
  }
}
