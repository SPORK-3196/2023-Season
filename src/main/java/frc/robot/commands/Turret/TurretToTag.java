package frc.robot.commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;

public class TurretToTag extends CommandBase {
    double pos;
    Turret turret;
    double speed;
    PIDController turretController;

    public TurretToTag(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.turretMotor.stopMotor();
        turretController = new PIDController(.08, 0, 0);
        pos = RobotContainer.aprilYaw;
        if(RobotContainer.isCube)
            speed = turretController.calculate(pos, 0);
        else 
            speed = turretController.calculate(pos, 3);
    }
    
    @Override
    public void execute() {
        turret.moveMotor(speed);
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
