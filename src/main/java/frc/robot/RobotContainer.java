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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Claw.CloseClawCone;
import frc.robot.commands.Claw.CloseClawCube;
import frc.robot.commands.Claw.ReleasePiece;
import frc.robot.commands.Drivetrain.DistanceToVisionTarget;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.commands.Drivetrain.FindAndRunTarget;
import frc.robot.commands.Drivetrain.TracktoVisionTarget;
import frc.robot.commands.Drivetrain.Turn90LeftDegrees;

import frc.robot.commands.TurretDrive;
import frc.robot.commands.Autonomous.SimpleAuto;
import frc.robot.commands.Autonomous.Positions.ArmPosition;
import frc.robot.commands.Autonomous.Positions.ElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private EveryClaw claw = new EveryClaw();
    public static Drivetrain drivetrain = new Drivetrain();
    public Arm arm = new Arm();
    public Lift lift = new Lift();
    private Turret turret = new Turret();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);
    private TurretDrive turretRotate = new TurretDrive(turret);

    private ElevatorPosition moveElevator = new ElevatorPosition(lift);
    private ArmPosition moveArm = new ArmPosition(arm);

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

    public static double LJSY_Arm = armController.getLeftY();

    public static double RJSX_Arm = armController.getRightX();

    public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
    public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
    public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
    public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value);

    public static JoystickButton LBP_Arm = new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
    public static JoystickButton RBP_Arm = new JoystickButton(armController, XboxController.Button.kRightBumper.value);

    public static JoystickButton A_Prim = new JoystickButton(primaryController, XboxController.Button.kA.value);
    public static JoystickButton B_Prim = new JoystickButton(primaryController, XboxController.Button.kB.value);
    public static JoystickButton Y_Prim = new JoystickButton(primaryController, XboxController.Button.kY.value);

    public static int elevatorSetPos = 0;
    public static int shoulderSetPos = 0;
    public static int elbowSetPos = 0;

    public double currentElevatorPosition;

    public static double currentElbowSetPos;
    public static double currentShoulderSetPos;

    public static boolean isCube = false;
    public static boolean pickUp = true;

    public RobotContainer() {
        configureButtonBindings();
        turret.setDefaultCommand(turretRotate);
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new ReleasePiece(claw));
        lift.setDefaultCommand(moveElevator);
        arm.setDefaultCommand(moveArm);
        autoChooser.setDefaultOption("Basic Auto", new SimpleAuto(drivetrain, arm, lift));
    }

    public void configureButtonBindings() {
            //Station Pos Pickup Position

        if (getPickUp() == true && OI.XboxController.X2_DPad == 0) {
            setElevatorSetPoint(Constants.LiftConstants.liftStationTick);
        }
                //Low Pos Pickup Position

        if (getPickUp() == true && OI.XboxController.X2_DPad == 180) {
            setElevatorSetPoint(Constants.LiftConstants.liftPickUpTick);
        }
        //High Rung Position

        // if (getPickUp() == false && OI.XboxController.X2_DPad == 0) {
        //     setElevatorSetPoint(Constants.LiftConstants.liftTopTick);
        // }
        
        //Low Rung Position

        if (getPickUp() == false && OI.XboxController.X2_DPad == 180) {
            setElevatorSetPoint(Constants.LiftConstants.liftMidTick);
        }
        //Mid Rung Position

        if (getPickUp() == false && OI.XboxController.X2_DPad == 270) {
            setElevatorSetPoint(Constants.LiftConstants.liftMidTick);
        }
        //Rest Position
                
        if (OI.XboxController.X2_DPad == 90) {
            setElevatorSetPoint(Constants.LiftConstants.liftRestTick);
        }

        B_Arm.onTrue(Commands.runOnce(() -> changePickUp()));

        if (isCube == false)
            X_Arm.whileTrue(new CloseClawCone(claw));
        if (isCube == true)
            X_Arm.whileTrue(new CloseClawCube(claw));

        LBP_Arm.onTrue(Commands.runOnce(() -> changeCube()));
        RBP_Arm.onTrue(Commands.runOnce(() -> changeCone()));

        A_Prim.whileTrue(new TracktoVisionTarget(drivetrain));
        B_Prim.onTrue(new DistanceToVisionTarget(drivetrain));
        Y_Prim.onTrue(new FindAndRunTarget(drivetrain));

    }

    public static Command trajectory() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("StraightPath",
                .75, .25);
        return PathGenerator.generateRamseteCommand(trajectory, drivetrain)
                .andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }

    public Command getSelected() {
        return autoChooser.getSelected();
    }

    public static PhotonPipelineResult pipelineResult(PhotonCamera camera) {
        return camera.getLatestResult();
    }

    public static boolean hasTargets(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    public static double getCamYaw(PhotonTrackedTarget target) {
        return target.getYaw();
    }

    public static double getCamPitch(PhotonTrackedTarget target) {
        return target.getPitch();
    }

    public static double distanceToVisionXTargetMeters() {
        return aprilX;
    }

    public static double distanceToVisionYTargetMeters() {
        return aprilY;
    }

    public static Transform3d distanceToVisionPose(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget();
    }

    public static int getFudicialID(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }

    public static double getPoseX() {
        return drivetrain.getPose().getX();
    }

    public static double getPoseY() {
        return drivetrain.getPose().getY();
    }

    public static double getPoseRotation() {
        return drivetrain.getPose().getRotation().getDegrees();
    }

    public static void setElevatorSetPoint(int setPoint) {
        elevatorSetPos = setPoint;
    }

    public static int getElevatorSetPoint() {
        return elevatorSetPos;
    }

    public static int getShoulderSetPoint() {
        return shoulderSetPos;
    }

    public static int getElbowSetPoint() {
        return elbowSetPos;
    }

    public static void changePickUp() {
        pickUp = !pickUp;
    }

    public static boolean getPickUp() {
        return pickUp;
    }

    public static void changeCube() {
        RobotContainer.isCube = true;
    }

    public static void changeCone() {
        RobotContainer.isCube = false;
    }
    public double getLiftEncoderTick(){
        return lift.getEncoderTick();
    }

    public double getShoulderEncoderTick(){
        return arm.getShoulderTick();
    }
    
    public double getElbowEncoderTick(){
        return arm.getElbowTick();
    }

    public void resetLiftEncoderTick(){
        currentElbowSetPos = 0;
    }

    public void resetElbowEncoderTick(){
        currentElbowSetPos = 0;
    }

    public void resetShoulderTick(){
        currentShoulderSetPos = 0;
    }
}