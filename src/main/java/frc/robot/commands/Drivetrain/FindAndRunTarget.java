package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FindAndRunTarget extends CommandBase{
    Command command;
    Drivetrain drivetrain;
    
    public FindAndRunTarget( Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.rearLeft.configOpenloopRamp(0);
        drivetrain.frontLeft.configOpenloopRamp(0);
        drivetrain.rearRight.configOpenloopRamp(0);
        drivetrain.frontRight.configOpenloopRamp(0);
        command = SplineToVisionTarget.trajectory(drivetrain);
        if(command == null){
            this.cancel();
            return; 
        }
        
        command.initialize();
    }
    @Override
    public void execute(){
        command.execute();
    }
    @Override
    public void end(boolean interrupted){
        command.cancel();
    }
    @Override
    public boolean isFinished(){
        return command.isFinished();
    }
}
