package frc.robot.commands.Drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PathGenerator;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SplineToVisionTarget {
    public static double dToTarget, l, x0, y0, x1, y1, x2, y2, heading, targetRot0;
    public static Command trajectory(Drivetrain drivetrain){
        drivetrain.resetOdometry();

        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    Constants.DrivetrainConstants.m_2022ksVolts,
                    Constants.DrivetrainConstants.m_2022kvVoltSecondsPerMeter,
                    Constants.DrivetrainConstants.m_2022kaVoltSecondsSquaredPerMeter),
                Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics,
                10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.DrivetrainConstants.maxSpeed,
                    Constants.DrivetrainConstants.kMaxAccelerationMetersPerSecSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        
        x0 = RobotContainer.distanceToVisionXTargetMeters();
        y0 = RobotContainer.distanceToVisionYTargetMeters();
        targetRot0 = RobotContainer.getCamYaw(RobotContainer.bResult);
        
        dToTarget = Math.sqrt(Math.pow(x0 - 1.5, 2) + Math.pow(y0, 2));
        heading = Units.radiansToDegrees(Math.atan2(y0,x0)) - targetRot0;

        Pose2d robPose = new Pose2d(x0, y0, new Rotation2d(heading));
        drivetrain.m_odometry.resetPosition(Drivetrain.gyroscope.getRotation2d() , 0, 0, robPose);

        l = dToTarget * 0.2;
        x1 = (l *Math.cos(Units.degreesToRadians(180 - heading))) + x0;
        y1 = -(l * Math.sin(Units.degreesToRadians(180 - heading))) + y0;
        x2 = 1.5 + (l);
        y2 = 0;

        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("x0: " + x0 + ", " + " y0: " + y0 + 
            ", " + " dToTarget: " + dToTarget + ", " + " heading: " +
             heading + " l: " + l + " x1: " + x1 +
              " y1: " + y1 + " x2: " + x2 + " y2: " + y2);
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        System.out.println("---------------------------------------------");
        
        // Trajectory to April Tag.  All units in meters.
        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at pose distance from the April Tag
                new Pose2d(x0, y0, 
                            new Rotation2d(heading)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(x1, y1), new Translation2d(x2, y2)),
                // End 1.5 meters straight away from the April Tag, facing towards it.
                new Pose2d(1.5, 0, new Rotation2d(0)),
                // Pass config
                config);
            
        return PathGenerator.generateRamseteCommand(trajectory, drivetrain).andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}