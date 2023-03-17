package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoArmToPickup extends CommandBase{
    Timer timer;
    public AutoArmToPickup(){
    
    }
    @Override
    public void initialize(){
        timer.reset();
        RobotContainer.setElbowSetPoint(Constants.ArmConstants.pickUpElbowTick);
        RobotContainer.setShoulderSetPoint(Constants.ArmConstants.pickUpShoulderTick);
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