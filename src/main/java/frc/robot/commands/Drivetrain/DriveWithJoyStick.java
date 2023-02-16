package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Robot;
import frc.robot.Variables;

public class DriveWithJoyStick extends CommandBase {
    private final Drivetrain drivetrain;
    double speedFiltered;
    double rotationFiltered;

    DifferentialDrive.WheelSpeeds wheelSpeeds;

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

        drivetrain.differentialDrive.setDeadband(.08);
    }
    
    @Override
    public void execute() {
        speedFiltered = Variables.XboxController.X1_LJX * 2/3;
        rotationFiltered = Variables.XboxController.X1_LJY * 2/3;

        drivetrain.arcadeDrive(speedFiltered, -rotationFiltered);
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