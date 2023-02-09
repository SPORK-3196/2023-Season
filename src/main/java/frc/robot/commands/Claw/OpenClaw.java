package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase{
    public Claw claw;

    public OpenClaw(Claw claw){
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize(){
        claw.openPistons();
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
