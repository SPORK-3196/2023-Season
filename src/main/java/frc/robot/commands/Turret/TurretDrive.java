package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretDrive extends CommandBase{
    private Turret turret;
    public TurretDrive(Turret turret){
        this.turret = turret;
        addRequirements(turret);
    }
    @Override
    public void initialize(){

    }
    public void execute(){

    }
    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return false;
    }
}
