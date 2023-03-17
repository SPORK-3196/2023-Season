package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBackwardsAuto extends CommandBase {
    Drivetrain drivetrain;

    public DriveBackwardsAuto(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){

    }

    @Override 
    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return false;
    }
}
