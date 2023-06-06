package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Claw.CloseClaw;
import frc.robot.commands.Claw.ReleasePiece;
import frc.robot.commands.Drivetrain.BrakeModeDrive;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
import frc.robot.commands.Drivetrain.TurnSetDegrees;
import frc.robot.commands.Lighting.LightingControl;
import frc.robot.commands.Turret.TurretToCenter;
import frc.robot.commands.Turret.TurretToTag;
import frc.robot.commands.TurretDrive;
import frc.robot.commands.Arm.TurnArmOff;
import frc.robot.commands.Autonomous.AutoHighChargeStation;
import frc.robot.commands.Autonomous.HighRungAuto;
import frc.robot.commands.Autonomous.HighRungPlace;
import frc.robot.commands.Autonomous.MiddleRungAuto;
import frc.robot.commands.Autonomous.Cube.TrackToCube;
import frc.robot.commands.Autonomous.Positions.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private static EveryClaw claw = new EveryClaw();
    public static Drivetrain drivetrain = new Drivetrain();
    public Arm arm = new Arm();
    private Turret turret = new Turret();
    private Lighting lighting = new Lighting();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);
    private TurretDrive turretRotate = new TurretDrive(turret);

    private ArmPosition moveArm = new ArmPosition(arm);

    public static PhotonCamera aprilTagCam = new PhotonCamera("Global_Shutter_Elevator");
    public static PhotonCamera raspiCam = new PhotonCamera("mmal_service_16.1");

    public static double aprilYaw, piYaw = 0;

    public static PhotonPipelineResult result, piResult;
    public static PhotonTrackedTarget bResult, piBResult;
    public static double aprilX, piX;
    public static double aprilY, piY;

    public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public static XboxController primaryController = new XboxController(0);
    public static XboxController armController = new XboxController(1);

    public static double LJSX_Primary = primaryController.getLeftX();
    public static double LJSY_Primary = primaryController.getLeftY();

    public static double LJSY_Arm = armController.getLeftY();

    public static double RJSX_Arm = armController.getRightX();
    public static double RJSY_Arm = armController.getRightY();

    public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
    public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
    public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
    public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value);

    public static JoystickButton LBP_Arm = new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
    public static JoystickButton RBP_Arm = new JoystickButton(armController, XboxController.Button.kRightBumper.value);

    public static JoystickButton A_Prim = new JoystickButton(primaryController, XboxController.Button.kA.value);
    public static JoystickButton B_Prim = new JoystickButton(primaryController, XboxController.Button.kB.value);
    public static JoystickButton X_Prim = new JoystickButton(primaryController, XboxController.Button.kX.value);

    public static JoystickButton Y_Prim = new JoystickButton(primaryController, XboxController.Button.kY.value);
    public static JoystickButton LJSButton = new JoystickButton(armController, XboxController.Button.kLeftStick.value);
    public static JoystickButton L_Start = new JoystickButton(armController, XboxController.Button.kStart.value);

    public static double R_TPrimary = primaryController.getRightTriggerAxis();


    public static double elevatorSetPos = 0;
    public static boolean elevatorUpdated = false;

    public static double shoulderSetPos = 0;
    public static double elbowSetPos = 0;

    public double elbowAngle = 0; 
    public double shoulderAngle = 0;

    public double currentElevatorPosition;
    public static double currentElbowPos = 0;
    public static double currentShoulderPos = 0;

    public static boolean isCube = true;
    public static boolean isHigh = true;

    public static boolean isElbowSwitchHit = false;
    public static boolean isShoulderSwitchHit = false;

    public RobotContainer() {
        configureButtonBindings();
        turret.setDefaultCommand(turretRotate);
        drivetrain.setDefaultCommand(joystickDrive);
        arm.setDefaultCommand(moveArm);
        lighting.setDefaultCommand(new LightingControl(lighting));

        autoChooser.addOption("Mid Rung Taxi", new MiddleRungAuto(drivetrain, claw));
        autoChooser.setDefaultOption("High Rung Taxi", new HighRungAuto(drivetrain, claw));
        autoChooser.addOption("High Rung Place", new HighRungPlace(drivetrain, claw));
        autoChooser.addOption("High Rung Charge", new AutoHighChargeStation(drivetrain, claw));
        autoChooser.addOption("Straight Test", trajectory());
        
    }

    public void configureButtonBindings() {
            //Station Pos Pickup Position

        X_Arm.whileTrue(new ReleasePiece(claw));
        A_Arm.whileTrue(new CloseClaw(claw));
        Y_Arm.onTrue(Commands.runOnce(() -> setSetPointsToPickUp()));

        LBP_Arm.onTrue(Commands.runOnce(() -> changePosPlaceHigh()));
        RBP_Arm.onTrue(Commands.runOnce(() -> changePosPlaceMid()));

        L_Start.whileTrue(new TurretToTag(turret));
        B_Arm.onTrue(new TurretToCenter(turret));

        LJSButton.whileTrue(new TurnArmOff(arm));

        // A_Prim.whileTrue(new TracktoVisionTarget(drivetrain));
        // B_Prim.onTrue(new DistanceToVisionTarget(drivetrain));
        Y_Prim.onTrue(new TurnSetDegrees(drivetrain, 180));

        X_Prim.whileTrue(new TrackToCube(drivetrain));

        A_Prim.whileTrue(new BrakeModeDrive(drivetrain));

    }

    public static Command trajectory() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Aroosh",
                1, .1);
        return PathGenerator.generateRamseteCommand(drivetrain, claw).fullAuto(trajectory).beforeStarting(new InstantCommand(drivetrain::resetOdometry, drivetrain)).beforeStarting(new InstantCommand(drivetrain::zeroGyro, drivetrain));
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
        try {
            return target.getYaw();
        } catch(Exception e) {
            return 0;
        }
        
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
        try {
            return target.getBestCameraToTarget();
        } catch(Exception e) {
            return new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0));
        }
        
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

    public static void setElevatorSetPoint(double setPoint) {
        elevatorUpdated = true;
        elevatorSetPos = setPoint;
    }

    public static double getElevatorSetPoint() {
        return elevatorSetPos;
    }

    public static double getShoulderSetPoint() {
        return shoulderSetPos;
    }

    public static double getElbowSetPoint() {
        return elbowSetPos;
    }

    public static void changePosPlaceHigh() {
        isHigh = true;
    }
    public static void changePosPlaceMid() {
        isHigh = false;
    }
    public static boolean getPosPlace() {
        return isHigh;
    }
    public static void changeCube() {
        RobotContainer.isCube = true;
    }

    public static void changeCone() {
        RobotContainer.isCube = false;
    }
    // public double getLiftEncoderTick(){
    //     return lift.getEncoderTick();
    // }

    public double getShoulderEncoderTick(){
        return arm.getShoulderTick();
    }
    
    public double getElbowEncoderTick(){
        return arm.getElbowTick();
    }

    // public void resetLiftEncoderTick(){
    //     lift.liftEncoder.setPosition(0);
    // }

    public void resetElbowEncoderTick(){
        arm.elbowEncoder.setPosition(0);
    }

    public void resetShoulderTick(){
        arm.shoulderEncoder.setPosition(0);
    }

    public static void setShoulderSetPoint(double setPoint){
        shoulderSetPos = setPoint;
    }
    public static void setElbowSetPoint(double setpoint){
        
        elbowSetPos = setpoint;
    }

    public double getShoulderAngle(){
        return arm.getShoulderAngle();
    }

    public double getElbowAngle(){
        return arm.getElbowAngle();
    }
    public boolean isShoulderLimitPressed(){
        return arm.isResetShoulder();
    }
    public void setSetPointsToPickUp(){
        if(isCube){
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.pickUpElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.pickUpShoulderTick);
        }
        else{
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.pickUpStationElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.pickUpStationShoulderTick);
        }
    }
    public double getTurretPos(){
        return turret.getTurretTick();
    }
    public static double getGyroPitch(){
        return Drivetrain.getGyroPitch();
    }
    public static Rotation2d getGyroYaw(){
        return Drivetrain.gyroscope.getRotation2d();
    }
    public static Pose2d getDrivetrainOdometry(){
        return drivetrain.m_odometry.getPoseMeters();
    }
}