package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClawCone extends CommandBase{
    public Claw claw;

    public CloseClawCone(Claw claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.closePiston1();
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
