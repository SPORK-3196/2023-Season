package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryClaw;

public class CloseClawCone extends CommandBase{
    public EveryClaw claw;

    public CloseClawCone(EveryClaw claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.runMotor(.5);
    }
    @Override
    public void execute(){
    }
    @Override 
    public void end(boolean isFinished){

    }
    public boolean isFinished(){
        return false;
    }
}
