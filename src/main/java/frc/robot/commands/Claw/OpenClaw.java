package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryClaw;

public class OpenClaw extends CommandBase{
    public EveryClaw claw;

    public OpenClaw(EveryClaw claw2){
        this.claw = claw2;

        addRequirements(claw2);
    }

    @Override
    public void initialize(){
      //  claw.openPistons();
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
