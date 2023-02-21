
package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RunArm extends CommandBase {
  Arm arm;
  public RunArm(Arm arm) {
    this.arm = arm;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    addRequirements(arm);
  }
    public void initialize() {
      }
    


  // Called repeatedly when this Command is scheduled to run
 //TODO old code
  public void execute() {
    //double armSpeed = -0.4*Robot.controller1.getRawAxis(1);
    //double wristSpeed = 0.4*Robot.controller1.getRawAxis(5);
    //double wristSpeed = Robot.controller1.getAButton() ? 0.4 : Robot.controller1.getBButton() ? -0.4 : 0;

    //Robot.arm.armMotor.set(armSpeed);
    //Robot.arm.wristMotor.set(wristSpeed);

  /*   double ShoulderOffset = 3.0;
    double ElbowOffset = 4.0;

    Robot.arm.ShoulderEncoder = Robot.arm.ShoulderMotor.getEncoder().getPosition();
    Robot.arm.ElbowEnoder = Robot.arm.ElbowMotor.getEncoder().getPosition();

    if(Robot.armController.getYButtonPressed()) {
      Robot.arm.shoulderOut = Robot.arm.shoulderOut;
    }

    if((boolean) Robot.arm.shoulderOut) {
      Robot.arm.ShoulderPID.setOutputRange(-0.15, 0.15);
      Robot.arm.ElbowPID.setOutputRange(-0.25, 0.25);

      if(Robot.armController.getAButton()) {
          Robot.arm.ShoulderPID.setReference((((Robot.lift.getEncoder() / 30000.0) - 1.0) * 16.9), CANSparkMax.kPosition);
          Robot.arm.ElbowPID.setReference((((Robot.arm.ShoulderEncoder + 16.9) * (7.6/16.9)) - 7.6), CANSparkMax.kPosition);
      } else if(Robot.armController.getBumper(GenericHID.kLeft)) {
        Robot.arm.ShoulderPID.setReference((((Robot.lift.getEncoder() / 30000.0) - 1.0) * 16.9) + ShoulderOffset, CANSparkMax.kPosition);
        Robot.arm.ElbowPID.setReference((((Robot.arm.ShoulderEncoder + 16.9) * (10.6/16.9)) - 10.6) + ElbowOffset, CANSparkMax.kPosition);
      } else {
        Robot.arm.ShoulderPID.setReference(ElbowOffset, CANSparkMax.Position)//setReference((((Robot.lift.getEncoder() / 30000.0) - 1.0) * 16.9), ControlType.kPosition);
        Robot.arm.ElbowPID.setReference((((Robot.arm.ShoulderEncoder + 16.9) * (10.6/16.9)) - 10.6), CANSparkMax.kPosition);
      } 
    } else {
      Robot.arm.ShoulderPID.setOutputRange(-0.4, 0.4);
      Robot.arm.ElbowPID.setOutputRange(-0.4, 0.4);

      if(Robot.lift.getEncoder() < 15000) {
        Robot.arm.ShoulderPID.setReference(0, CANSparkMax.kPosition); 
        Robot.arm.ElbowPID.setReference(0, CANSparkMax.kPosition);
      } else {
        Robot.arm.ShoulderPID.setReference(-1.69, CANSparkMax.kPosition); 
        Robot.arm.ElbowPID.setReference(-1.06, CANSparkMax.kPosition);
      }*/
    }

    //Robot.arm.armPID.setReference(-15, ControlType.kPosition);

    /*System.out.print(Robot.arm.armEncoder);
    System.out.print("\t\t\t");
    System.out.println(Robot.arm.wristEncoder);*/

    //System.out.println(((Robot.arm.armEncoder + 15.0) * (9.0/15.0)) - 9f);


  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true

  protected void end() {
    //System.out.println("Finished using manual PID arm control...");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

  protected void interrupted() {
  }
}