package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomous.Cone.PickupConeStation;
import frc.robot.commands.Autonomous.Cube.PickupCubeStation;
import frc.robot.commands.Autonomous.Positions.Square;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.commands.Drivetrain.TracktoTag;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private Claw claw = new Claw();
    private static Drivetrain drivetrain = new Drivetrain(); 
    private Arm arm = new Arm(); 
    private Lift lift= new Lift();
    private Turret turret = new Turret();

    public static PhotonCamera aprilTagCam = new PhotonCamera("Global_Shutter_Elevator");
    public static PhotonCamera primaryCamera = new PhotonCamera("Primary Camera");
    public static PhotonCamera backupCamera = new PhotonCamera("Backup Camera");
    public static double aprilYaw = 0;

    public static PhotonPipelineResult result = aprilTagCam.getLatestResult();
    public static PhotonTrackedTarget bResult = result.getBestTarget();

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);

    public static XboxController primaryController = new XboxController(0);
    public static XboxController armController = new XboxController(1);
 
    public static double LJSX_Primary = primaryController.getLeftX();
    public static double LJSY_Primary = primaryController.getLeftY();
    
    public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
    public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
    public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
    public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value); 

    public static JoystickButton A_Prim = new JoystickButton(primaryController, XboxController.Button.kA.value);


    public RobotContainer(){
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
        autoChooser.addOption("Straight Traj", trajectory());
        autoChooser.setDefaultOption("Turn 45 Degrees", new Square(drivetrain));
    }

    public void configureButtonBindings() {
        if(Variables.XboxController.X2_DPad == 0) A_Arm.onTrue(new PickupConeStation(arm, lift, claw));
        if(Variables.XboxController.X2_DPad == 0) X_Arm.onTrue(new PickupCubeStation(arm, lift, claw));
        
        A_Prim.whileTrue(new TracktoTag(drivetrain));
    }   
     
    public static Command trajectory(){

        PathPlannerTrajectory trajectory = 
        PathPlanner.loadPath("Straight",
         1, .25);

        RamseteCommand ramseteCommand = 
            new RamseteCommand(trajectory, 
            drivetrain::getPose, 
            new RamseteController(2, .7), 
            drivetrain.m_feedforward,
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
        return autoChooser.getSelected();
    } 

    public static PhotonPipelineResult pipelineResult(PhotonCamera camera){
        return camera.getLatestResult();
    }
    public static boolean hasTargets(PhotonPipelineResult result){
        return result.hasTargets();
    }
    public static double getCamYaw(PhotonTrackedTarget target){
        return target.getYaw();
    }

}