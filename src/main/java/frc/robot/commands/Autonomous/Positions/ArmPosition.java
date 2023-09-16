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

     if(arm.getElbowTick() > 2 && RobotContainer.shoulderSetPos > -6)//danger
     {
          //System.out.println("Danger to arm");
          arm.shoulderController.setReference(-7, CANSparkMax.ControlType.kPosition); //TODO change the setpoint
     }
     else{
          //System.out.println("slider is OK");
          arm.shoulderController.setReference(RobotContainer.shoulderSetPos, CANSparkMax.ControlType.kPosition);
     }
     if(arm.getShoulderTick() > -6 && RobotContainer.elbowSetPos >= 0 )
     {
          //System.out.println("Shoulder to low");
          arm.elbowController.setReference(0, CANSparkMax.ControlType.kPosition);//TODO set setpoint
     }
     else{
          //System.out.println("Shoulder is OK");
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
