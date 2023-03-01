package frc.robot;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomous.HighRung;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.Drivetrain.DistanceToVisionTarget;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.commands.Drivetrain.FindAndRunTarget;
import frc.robot.commands.Drivetrain.TracktoVisionTarget;
import frc.robot.commands.Drivetrain.Turn90LeftDegrees;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private Claw claw = new Claw();
    public static Drivetrain drivetrain = new Drivetrain(); 
    private Arm arm = new Arm(); 
    private Lift lift= new Lift();
    private Turret turret = new Turret();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);

    public static PhotonCamera aprilTagCam = new PhotonCamera("Global_Shutter_Elevator");
    public static PhotonCamera raspiCam = new PhotonCamera("mmal_service_16.1");
    public static double aprilYaw = 0;

    public static PhotonPipelineResult result;
    public static PhotonTrackedTarget bResult;
    public static PhotonTrackedTarget rasbResult;
    public static double aprilX;
    public static double aprilY;

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static XboxController primaryController = new XboxController(0);
    public static XboxController armController = new XboxController(1);
 
    public static double LJSX_Primary = primaryController.getLeftX();
    public static double LJSY_Primary = primaryController.getLeftY();
    
    public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
    public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
    public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
    public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value); 

    public static JoystickButton A_Prim = new JoystickButton(primaryController, XboxController.Button.kA.value);
    public static JoystickButton B_Prim = new JoystickButton(primaryController, XboxController.Button.kB.value);
    public static JoystickButton Y_Prim = new JoystickButton(primaryController, XboxController.Button.kY.value);


    public RobotContainer(){
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
        autoChooser.addOption("Straight Traj", trajectory());
        autoChooser.setDefaultOption("Turn left 90 degrees", new Turn90LeftDegrees(drivetrain));
    }

    public void configureButtonBindings() {

        // if(OI.XboxController.X2_DPad == 0) A_Arm.onTrue(new HighRung());
        // if(OI.XboxController.X2_DPad == 0) X_Arm.onTrue(new PickupCubeStation(arm, lift, claw));
        
        
        A_Prim.whileTrue(new TracktoVisionTarget(drivetrain));
        B_Prim.onTrue(new DistanceToVisionTarget(drivetrain));
        Y_Prim.onTrue(new FindAndRunTarget(drivetrain));
    }   

    public static Command trajectory(){
        PathPlannerTrajectory trajectory = 
        PathPlanner.loadPath("StraightPath",
         .75, .25);
        return PathGenerator.generateRamseteCommand(trajectory, drivetrain)
            .andThen(() -> drivetrain.tankDriveVolts(0, 0));
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
    public static double getCamPitch(PhotonTrackedTarget target){
        return target.getPitch();
    }
    public static double distanceToVisionXTargetMeters(){
        return aprilX;
    }
    public static double distanceToVisionYTargetMeters(){
        return aprilY;
    }
    public static Transform3d distanceToVisionPose(PhotonTrackedTarget target){
        return target.getBestCameraToTarget();
    }
    public static int getFudicialID(PhotonTrackedTarget target){
        return target.getFiducialId();
    }
    public static double getPoseX(){
        return drivetrain.getPose().getX();
    }
    public static double getPoseY(){
        return drivetrain.getPose().getY();
    }
    public static double getPoseRotation(){
        return drivetrain.getPose().getRotation().getDegrees();

    }
}