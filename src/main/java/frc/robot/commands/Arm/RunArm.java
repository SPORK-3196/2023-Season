
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
    
  public void execute() {

  }
  
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