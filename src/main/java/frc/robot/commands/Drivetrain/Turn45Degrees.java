package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn45Degrees extends CommandBase  {
    Drivetrain drivetrain;
    double kP = .5;
    
    public Turn45Degrees(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        drivetrain.zeroGyro();
    }
    @Override
    public void execute(){ 
        double error = drivetrain.gyroscope.getRate();
        
        drivetrain.arcadeDrive(0, .5 - kP * error);
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.arcadeDrive(0, 0);
    }
    public boolean isFinished(){
        return drivetrain.gyroscope.getYaw() == 45;
    }
}
