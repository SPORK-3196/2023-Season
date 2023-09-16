package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EveryClaw;

public class CloseClaw extends CommandBase{
    public EveryClaw claw;

    public CloseClaw(EveryClaw claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }
    @Override
    public void execute(){
        if(RobotContainer.isCube)
            claw.runMotor(-.8);
            
        else
            claw.runMotor(.8);
    }
    @Override 
    public void end(boolean isFinished){
        claw.stopMotor();
    }
    public boolean isFinished(){
        return false;
    }
}
