package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;


public class ReZero extends CommandBase{
    public Arm arm;

    public ReZero(Arm arm){
    this.arm = arm;

    addRequirements(arm);
    }

    @Override
    public void initialize(){
        arm.elbowMotor.set(0);
    }
    @Override
    public void execute(){
        arm.elbowMotor.set(-0.1);
    }
    @Override 
    public void end(boolean isFinished){
        arm.elbowMotor.set(0);;
    }
    public boolean isFinished(){
        return false;
    }
}
