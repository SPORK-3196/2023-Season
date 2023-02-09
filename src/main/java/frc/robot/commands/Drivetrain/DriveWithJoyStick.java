package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;

public class DriveWithJoyStick extends CommandBase {
    private Drivetrain drivetrain;
    double speedFiltered;
    double rotationFiltered;

    public DriveWithJoyStick(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    }
    
    @Override
    public void execute() {
        speedFiltered = Robot.filter.calculate(Robot.LJSY_Primary);
        rotationFiltered = Robot.filter.calculate(Robot.LJSX_Primary);

        drivetrain.arcadeDrive(speedFiltered, rotationFiltered);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
        
        drivetrain.differentialDrive.setDeadband(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}