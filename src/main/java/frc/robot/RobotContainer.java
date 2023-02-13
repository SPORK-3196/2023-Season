package frc.robot;

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
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Turret;
import frc.robot.commands.Drivetrain.StraightTrajectory;

public class RobotContainer {
    private Claw claw = new Claw();
    private Drivetrain drivetrain = new Drivetrain();
    private Arm arm = new Arm(); 
    private Lift lift= new Lift();
    private Turret turret = new Turret();

    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);
    public RobotContainer(){
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
        autoChooser.setDefaultOption("Straight Traj 6.49m", new StraightTrajectory(drivetrain));
    }

    public void configureButtonBindings() {
    }
    public Command getSelected(){
        return autoChooser.getSelected();

    }   
}