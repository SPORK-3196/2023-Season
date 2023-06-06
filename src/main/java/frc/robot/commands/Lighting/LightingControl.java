package frc.robot.commands.Lighting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class LightingControl extends CommandBase {
    Lighting lights;

    public LightingControl(Lighting lights){
        this.lights = lights;
        
        addRequirements(lights);
    }

    @Override
    public void initialize(){
    }


    @Override
    public void execute(){
        
        if(DriverStation.isDisabled()){
            if(DriverStation.getAlliance().compareTo(DriverStation.Alliance.Blue) == 0){        
                lights.fullYellow();
            }
            else if(DriverStation.getAlliance().compareTo(DriverStation.Alliance.Red) == 0){
                lights.fullPurple();
            }
            else{
                lights.rainbowRun();
            }
        }

        if(DriverStation.isTeleopEnabled()){
            lights.FullRainbow();
        }
        if(DriverStation.isAutonomousEnabled()){
            lights.fullWhite();
        }

        lights.fullWhite();
        lights.start();
    }

    @Override
    public void end(boolean interrupted){
        lights.fullWhite();
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}