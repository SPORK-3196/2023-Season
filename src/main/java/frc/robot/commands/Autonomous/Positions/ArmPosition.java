package frc.robot.commands.Autonomous.Positions;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class ArmPosition extends CommandBase {
     Arm arm;

    public ArmPosition(Arm arm){
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
   public void initialize(){
        arm.elbowController.setReference(RobotContainer.elbowSetPos, CANSparkMax.ControlType.kPosition);
        arm.shoulderController.setReference(RobotContainer.shoulderSetPos, CANSparkMax.ControlType.kPosition);
   }

   @Override
   public void execute(){
        
   }

   @Override
   public void end(boolean isFinished){
        arm.turnElbowOff();
        arm.turnShoulderOff();
   }

   @Override
   public boolean isFinished(){
      return false;
   }

}
