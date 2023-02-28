package frc.robot;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class PathGenerator {
    
    public static RamseteCommand generateRamseteCommand(Trajectory trajectory, Drivetrain drivetrain){
        return new RamseteCommand( 
            trajectory, 
            drivetrain::getPose, 
            new RamseteController(2, .7), 
            drivetrain.m_feedforward,
            Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics,
            drivetrain::motorWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
            new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);
    }
}