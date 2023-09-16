package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryClaw;

public class ReleasePiece extends CommandBase{
    public EveryClaw claw;

    public ReleasePiece(EveryClaw claw2){
        this.claw = claw2;

        addRequirements(claw2);
    }

    @Override
    public void initialize(){
        claw.stopMotor();
    }
    @Override
    public void execute(){
        claw.runMotor(.1);
    }
    @Override 
    public void end(boolean isFinished){
        claw.runMotor(-0.03);
    }
    public boolean isFinished(){
        return false;
    }
}
