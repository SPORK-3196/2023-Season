package frc.robot.commands.Autonomous.Positions;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Lift;

public class ElevatorPosition extends CommandBase{
   Lift lift;
   public ElevatorPosition(Lift lift){
        this.lift = lift;

        addRequirements(lift);
   }

   @Override
   public void initialize(){
   }

   @Override
   public void execute(){
      lift.liftController.setReference(RobotContainer.elevatorSetPos, CANSparkMax.ControlType.kPosition);
   }

   @Override
   public void end(boolean isFinished){
      lift.stopLift();
   }

   @Override
   public boolean isFinished(){
      return false;
   }
}
