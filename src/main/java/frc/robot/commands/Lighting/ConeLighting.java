package frc.robot.commands.Lighting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lighting;

public class ConeLighting extends CommandBase {
    Lighting lights;
    Timer timer;

    public ConeLighting(Lighting lights){
        this.lights = lights;
        
        addRequirements(lights);
    }

    @Override
    public void initialize(){
        lights.fullWhite();
        lights.start();

        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        lights.fullYellow();
        lights.start();
    } 

    @Override
    public void end(boolean interrupted){
        lights.fullWhite();
    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(2);
    }
}