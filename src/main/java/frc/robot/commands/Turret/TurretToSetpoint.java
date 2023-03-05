package frc.robot.commands.Turret;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretToSetpoint extends CommandBase {
    double degreeSetPoint;
    Turret turret;

    public TurretToSetpoint(Turret turret, double degreeSetPoint) {
        this.turret = turret;
        this.degreeSetPoint = degreeSetPoint;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.turretController.setReference(degreeSetPoint, ControlType.kPosition);
    }
    
    @Override
    public void execute() {
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
