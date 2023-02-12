package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class StraightTrajectory extends CommandBase {
    public Drivetrain drivetrain;
    
    public StraightTrajectory(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    public Command trajectoryCommand() {

        PathPlannerTrajectory trajectory = 
            PathPlanner.loadPath("BStraightCommToRightCenter",
             4, 3);

        RamseteCommand ramseteCommand = 
            new RamseteCommand(trajectory, 
            drivetrain::getPose, 
            new RamseteController(2, .7), 
            new SimpleMotorFeedforward(Constants.DrivetrainConstants.m_2022ksVolts,
                Constants.DrivetrainConstants.m_2022kvVoltSecondsPerMeter,
                Constants.DrivetrainConstants.m_2022kaVoltSecondsSquaredPerMeter),
            Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics,
            drivetrain::motorWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
            new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);

        drivetrain.resetOdometry();
        
        return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }

    @Override
    public void initialize(){
        trajectoryCommand();

    }
    
    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
