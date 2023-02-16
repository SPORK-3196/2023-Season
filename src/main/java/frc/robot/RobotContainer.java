package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Autonomous.Cone.PickupConeStation;
import frc.robot.commands.Autonomous.Cube.PickupCubeStation;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;
import frc.robot.Robot;

public class RobotContainer {
    private Claw claw = new Claw();
    private static Drivetrain drivetrain = new Drivetrain();
    private Arm arm = new Arm(); 
    private Elevator elevator = new Elevator();
    private Turret turret = new Turret();

    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);

    public RobotContainer(){
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
        System.out.println("Got here-joystick");
        autoChooser.setDefaultOption("Straight Traj", trajectory());
    }

    public void configureButtonBindings() {
        if(Robot.dPad == 0) Robot.A_Arm.onTrue(new PickupConeStation(arm, elevator, claw));
        if(Robot.dPad == 0) Robot.X_Arm.onTrue(new PickupCubeStation(arm, elevator, claw));

    }   
     
    public static Command trajectory(){

        PathPlannerTrajectory trajectory = 
        PathPlanner.loadPath("Straight",
         1, .25);

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
    public Command getSelected(){
        System.out.println("Got here-getselected");
        return autoChooser.getSelected();
    }  
}