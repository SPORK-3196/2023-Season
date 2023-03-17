package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretToCenter extends CommandBase{
    Turret turret;
    PIDController turretController;
    double speed;
    public TurretToCenter(Turret turret){
        this.turret = turret;
        addRequirements(turret);
    }
    @Override
    public void initialize(){
        turretController = new PIDController(.08, 0, 0);
    }
    @Override
    public void execute(){
        speed = turretController.calculate(turret.getTurretTick(), 0);
    }
    @Override
    public void end(boolean interrupted){
        turret.turretMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        return turretController.atSetpoint();
    }
}
