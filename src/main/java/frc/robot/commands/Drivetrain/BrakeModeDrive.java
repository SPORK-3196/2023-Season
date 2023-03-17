package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class BrakeModeDrive extends CommandBase{
    Drivetrain drivetrain;
    double speedFiltered;
    double rotationFiltered;

    public BrakeModeDrive(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute(){
        speedFiltered = RobotContainer.LJSY_Primary * .65;
        rotationFiltered = RobotContainer.LJSX_Primary * .33;
        
        drivetrain.arcadeDriveAI(speedFiltered, rotationFiltered);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
