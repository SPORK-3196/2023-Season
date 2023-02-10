package frc.robot.subsystems;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.commands.LiftWithJoystick;

/**
 * Add your docs here.
 */
public class Lift extends PIDSubsystem {
  /**
   * Add your docs here.
   */

  public double offset = 0;

  //public PWMSparkMax liftMotor = new PWMSparkMax(10);
  public CANSparkMax liftMotor = new CANSparkMax(10, MotorType.kBrushless);

  //public Servo cameraServo = new Servo(0);

  public double getEncoder() {
    RelativeEncoder liftEncoder = liftMotor.getEncoder();
    return offset;
  }

  public void resetEncoder() {
    RelativeEncoder liftEncoder = liftMotor.getEncoder();
  }

  public void resetEncoderTo(int val) {
    //SparkMaxAbsoluteEncoder current = liftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    //current.getPosition();
    RelativeEncoder liftEncoder = liftMotor.getEncoder();
   // offset = liftMotor.getEncoder();
     ;
    //offset = liftMotor.getSelectedSensorValue() - val;
  }
  
  public Lift() {
    // Intert a subsystem name and PID values here
    super(new PIDController(.0010, 0, .001));
    //super("Lift", 0.0010, 0, 0.001);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    setSetpoint(0);
    disable();
  }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new LiftWithJoystick());
  }

  
  public double returnPIDInput() {
    return getEncoder();
  }

  public void usePIDOutput(double val) {
    liftMotor.set(-val);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    
  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}

  
/*private static double kDt = 0.02;
// discribes kDt
private final Encoder m_encoder = new Encoder(1, 2);
private final MotorController m_MotorController = new PWMSparkMax(1);
//encoder and Motor Controller
private final TrapezoidProfile.Constraints m_Constraints =
 new TrapezoidProfile.Constraints(1.75, 0.75);
 private final ProfiledPIDController m_Controller = 
 new ProfiledPIDController(1.3, 0.0, 0.7, m_Constraints, kDt);
 // i dont know what this means exactly 
 public Elevator() {
    m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
 }
 //TODO find the controll method for the elevator
/*  public CommandBase ElevatorTop(){
    return this.runOnce(() -> m_MotorController.set(1));
 }
 public CommandBase ElevatorLow(){
    return this.runOnce(() -> m_MotorController.set(0));
 }
 public CommandBase elevatorSpecifyed(){
    return this.runOnce(() -> m_encoder.setDistancePerPulse(Constants.kElevatorDistance));
 }*/
// elevator controll stuff