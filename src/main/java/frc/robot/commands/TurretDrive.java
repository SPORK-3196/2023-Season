package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.RobotContainer;

public class TurretDrive extends CommandBase{
    double speed;
    
    private Turret turret;
    public TurretDrive(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }
    
    @Override
    public void initialize() {
        turret.turretMotor.stopMotor();
    }
    
    @Override
    public void execute() {
        speed = RobotContainer.RJSX_Arm;
        turret.moveMotor(speed * .33);
    }

    @Override
    public void end(boolean interrupted) {
       turret.turretMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}