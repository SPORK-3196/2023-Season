package frc.robot.commands.Lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lift;

public class LiftWithJoystick extends CommandBase {
  Lift lift;
  double speed;
  public LiftWithJoystick(Lift lift){
    this.lift = lift;
    addRequirements(lift);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    lift.runLift(0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    speed = RobotContainer.LJSY_Arm * .33;
    lift.runLift(speed);
  
  }
  // Called once after isFinished returns true
  @Override
  public void end(boolean isFinished) {
    lift.stopLift();
  }

  @Override
  public boolean isFinished(){
    return false;
  }  
}