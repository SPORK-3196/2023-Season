package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;
import frc.robot.commands.Drivetrain.StraightTrajectory;

public class RobotContainer {
    private Claw claw = new Claw();
    private Drivetrain drivetrain = new Drivetrain();
    private Arm arm = new Arm(); 
    private Elevator elevator = new Elevator();
    private Turret turret = new Turret();
    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);
    public RobotContainer(){
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
        autoChooser.setDefaultOption("Straight Traj 6.49m", new StraightTrajectory(drivetrain));
    }
    // public void trajectoryCommand(){
    //     var autoVoltageConstraint = 
    //             new DifferentialDriveVoltageConstraint(
    //                 new SimpleMotorFeedforward(Constants.DrivetrainConstants.m_2022ksVolts, 
    //                     Constants.DrivetrainConstants.m_2022kvVoltSecondsPerMeter, 
    //                     Constants.DrivetrainConstants.m_2022kaVoltSecondsSquaredPerMeter), 
    //                     Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics, 10);
        
    //         TrajectoryConfig config = 
    //             new TrajectoryConfig(Constants.DrivetrainConstants.maxSpeed, 
    //             Constants.DrivetrainConstants.kMaxAccelerationMetersPerSecSquared)
    //             .setKinematics(Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics)
    //             .addConstraint(autoVoltageConstraint);
    // Trajectory trajectory =
    //             TrajectoryGenerator.generateTrajectory(
    //                 new Pose2d(0, 0, new Rotation2d(0)),
    //                 List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //                 new Pose2d(3, 0, new Rotation2d(0)),
    //                 config);
    // }

    public void configureButtonBindings() {

    }
    public Command getSelected(){
        return autoChooser.getSelected();
    }   
}