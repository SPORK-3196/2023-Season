package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
        if(RobotContainer.isCube)
            claw.runMotor(.8);
        else
            claw.runMotor(-.3);
    }
    @Override 
    public void end(boolean isFinished){
        claw.stopMotor();
    }
    public boolean isFinished(){
        return false;
    }
}
