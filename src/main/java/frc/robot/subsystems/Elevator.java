package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

/**
 * Add your docs here.
 */
public class Elevator extends PIDSubsystem {
  /**
   * Add your docs here.
   */

  public double offset = 0;

  //public PWMSparkMax liftMotor = new PWMSparkMax(10);
  public CANSparkMax liftMotor = new CANSparkMax(10, MotorType.kBrushless);
  public RelativeEncoder liftEncoder = liftMotor.getEncoder();
  //public Servo cameraServo = new Servo(0);

  public double getEncoder() {
    return offset;
  }

  public void resetEncoder() {
    liftEncoder.setPosition(0);
  }

  public void resetEncoderTo(int val) {

  }
  
  public Elevator() {
    super(new PIDController(.0010, 0, .001));
    setSetpoint(0);
    disable();
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new LiftWithJoystick());
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