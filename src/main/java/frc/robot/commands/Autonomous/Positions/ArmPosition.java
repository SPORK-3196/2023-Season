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

   }

@Override
   public void execute(){

     if(arm.getElbowTick() > 1000 && RobotContainer.shoulderSetPos < 2000)//danger
     {
          arm.shoulderController.setReference(20, CANSparkMax.ControlType.kPosition); //TODO change the setpoint
     }
     else{
          arm.shoulderController.setReference(RobotContainer.shoulderSetPos, CANSparkMax.ControlType.kPosition);
     }
     if(arm.getShoulderTick() < 2000 && RobotContainer.elbowSetPos > 0 ){
          arm.elbowController.setReference(0, CANSparkMax.ControlType.kPosition);//TODO set setpoint
     }
     else{
          arm.elbowController.setReference(RobotContainer.elbowSetPos, CANSparkMax.ControlType.kPosition);
     }    
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
