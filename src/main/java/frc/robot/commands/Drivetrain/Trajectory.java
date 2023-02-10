package frc.robot.commands.Drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Trajectory extends CommandBase {
    public Drivetrain drivetrain;
    
    public Trajectory(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        // public Command trajectoryCommand(){
        //     var autoVoltageConstraint = 
        //         new DifferentialDriveVoltageConstraint(
        //             new SimpleMotorFeedforward(Constants.DrivetrainConstants.m_2022ksVolts, 
        //                 Constants.DrivetrainConstants.m_2022kvVoltSecondsPerMeter, 
        //                 Constants.DrivetrainConstants.m_2022kaVoltSecondsSquaredPerMeter), 
        //                 Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics, 10);
        
        //     TrajectoryConfig config = 
        //         new TrajectoryConfig(Constants.DrivetrainConstants.maxSpeed, 
        //         Constants.DrivetrainConstants.kMaxAccelerationMetersPerSecSquared)
        //         .setKinematics(Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics)
        //         .addConstraint(autoVoltageConstraint);

        //     Trajectory exampleTrajectory =
        //         TrajectoryGenerator.generateTrajectory(
        //             new Pose2d(0, 0, new Rotation2d(0)),
        //             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //             new Pose2d(3, 0, new Rotation2d(0)),
        //             config);

        //     RamseteCommand ramseteCommand = 
        //         new RamseteCommand(exampleTrajectory, 
        //         drivetrain::getPose, 
        //         new RamseteController(2, .7), 
        //         new SimpleMotorFeedforward(Constants.DrivetrainConstants.m_2022ksVolts,
        //             Constants.DrivetrainConstants.m_2022kvVoltSecondsPerMeter,
        //             Constants.DrivetrainConstants.m_2022kaVoltSecondsSquaredPerMeter),
        //         Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics,
        //         drivetrain::motorWheelSpeeds,
        //         new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
        //         new PIDController(Constants.DrivetrainConstants.m_2022kP, 0, 0),
        //         drivetrain::tankDriveVolts,
        //         drivetrain);
        // }
    }
}
