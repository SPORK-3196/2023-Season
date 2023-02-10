package frc.robot.commands;

import java.lang.module.ModuleDescriptor;
import java.lang.module.ModuleDescriptor.Requires;
import java.util.Set;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Lift;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class LiftWithJoystick extends CommandBase {
  public LiftWithJoystick(){
    Requires(Robot.lift);
  }

  private void Requires(Lift lift) {
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    Robot.lift.liftMotor.set(0);
  }

  // Called repeatedly when this Command is scheduled to run
  
  public void execute() {
    if(Robot.lift.getEncoder() > 20000) {
    } else if(Robot.lift.getEncoder() > 10000) {
    }
  

    double liftInput = - Robot.primaryController.getRawAxis(1);
    double liftSpeedCoef = -1.0;

    System.out.println(Robot.lift.getEncoder());
    if(Robot.primaryController.getXButtonPressed()) {
      Robot.arm.shoulder = false; // Cargo dropoff
      Robot.lift.setSetpoint(6000);
      Robot.lift.enable();
    } else if(Robot.primaryController.getBButtonPressed()) {
      Robot.arm.elbow = true; // Hatch pickup
      Robot.lift.setSetpoint(3000);
      Robot.lift.enable();
    }

    if(Math.abs(liftInput) > 0.08) {
      ((PIDSubsystem) Robot.liftMotor).disable();
      if(liftInput < 0.0) {
        if(((Lift) Robot.liftMotor).getEncoder() < 10000) {
          liftSpeedCoef = -0.4;
        }
        if(((Lift) Robot.liftMotor).getEncoder() < 1000) {
          liftSpeedCoef = -0.0;
        }
      }

  
      //Robot.lift.setSetpoint(Robot.lift.getEncoder());
    }

    
    //System.out.println(Robot.lift.getPIDController().getError());
  }


  // Make this return true when this Command no longer needs to run execute()

  public boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  
  protected void interrupted() {
  }

  @Override
  public Set<Subsystem> getRequirements() {
    // TODO Auto-generated method stub
    return null;
  }
}