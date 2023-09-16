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
        claw.runMotor(-.6);
    }
    @Override 
    public void end(boolean isFinished){
        claw.runMotor(-0.03);
    }
    public boolean isFinished(){
        return false;
    }
}
