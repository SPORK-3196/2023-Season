package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoMoveArmToTop extends CommandBase{
    Timer timer;
    public AutoMoveArmToTop(){
    
    }
    @Override
    public void initialize(){
        timer.reset();
        //RobotContainer.setElbowSetPoint(Constants.ArmConstants.midCubeElbowTick);
        RobotContainer.setShoulderSetPoint(Constants.ArmConstants.highCubeShoulderTick);
        //Timer.delay(1);
        RobotContainer.setElbowSetPoint(Constants.ArmConstants.highCubeElbowTick);
        timer.start();
    }
    @Override
    public void execute(){

    }

    public void end(boolean interrupted){
        timer.stop();
    }

    public boolean isFinished(){
        return timer.hasElapsed(3);
    }
}