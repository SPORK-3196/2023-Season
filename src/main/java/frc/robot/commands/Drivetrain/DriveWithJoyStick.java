package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

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

        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);

        drivetrain.differentialDrive.setDeadband(.08);
    }
    
    @Override
    public void execute() {
        speedFiltered = RobotContainer.LJSY_Primary * 0.67;
        rotationFiltered = RobotContainer.LJSX_Primary * 0.5;

        drivetrain.arcadeDrive(speedFiltered, -rotationFiltered);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
        
        drivetrain.differentialDrive.setDeadband(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}