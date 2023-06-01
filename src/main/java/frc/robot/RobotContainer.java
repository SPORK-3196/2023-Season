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
import frc.robot.commands.Claw.CloseClaw;
import frc.robot.commands.Claw.ReleasePiece;
import frc.robot.commands.Drivetrain.BrakeModeDrive;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;


import frc.robot.commands.TurretDrive;
import frc.robot.commands.Autonomous.HighRungAuto;
import frc.robot.commands.Autonomous.LowRungAuto;
import frc.robot.commands.Autonomous.MiddleRungAuto;
import frc.robot.commands.Autonomous.Positions.ArmPosition;
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
    public static double RJSY_Arm = armController.getRightY();

    public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
    public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
    public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
    public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value);

    public static JoystickButton LBP_Arm = new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
    public static JoystickButton RBP_Arm = new JoystickButton(armController, XboxController.Button.kRightBumper.value);

    public static JoystickButton A_Prim = new JoystickButton(primaryController, XboxController.Button.kA.value);
    public static JoystickButton B_Prim = new JoystickButton(primaryController, XboxController.Button.kB.value);
    public static JoystickButton Y_Prim = new JoystickButton(primaryController, XboxController.Button.kY.value);


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
    public static boolean pickUp = true;

    public static boolean isElbowSwitchHit = false;
    public static boolean isShoulderSwitchHit = false;

    public RobotContainer() {
        configureButtonBindings();
        turret.setDefaultCommand(turretRotate);
        drivetrain.setDefaultCommand(joystickDrive);
        arm.setDefaultCommand(moveArm);
        autoChooser.addOption("Mid Rung Auto", new MiddleRungAuto(drivetrain, claw));
        autoChooser.setDefaultOption("High Rung Auto", new HighRungAuto(drivetrain, claw));
        autoChooser.addOption("Low Rung Auto", new LowRungAuto(drivetrain, claw));
        
    }

    public void configureButtonBindings() {

        B_Arm.onTrue(Commands.runOnce(() -> changePickUp()));

            X_Arm.whileTrue(new ReleasePiece(claw));
            A_Arm.whileTrue(new CloseClaw(claw));
            Y_Arm.onTrue(Commands.runOnce(() -> setSetPointsToPickUp()));

        LBP_Arm.onTrue(Commands.runOnce(() -> changeCube()));
        RBP_Arm.onTrue(Commands.runOnce(() -> changeCone()));


        A_Prim.whileTrue(new BrakeModeDrive(drivetrain));

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
        lift.liftEncoder.setPosition(0);
    }

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
    public void resetElevatorIAccum(){
        lift.liftController.setIAccum(0);
    }
    public boolean isElevLimitPressed(){
        return lift.isResetLift();
    }
    public boolean isShoulderLimitPressed(){
        return arm.isResetShoulder();
    }
    public void setSetPointsToPickUp(){
        RobotContainer.setElbowSetPoint(Constants.ArmConstants.pickUpElbowTick);
        RobotContainer.setShoulderSetPoint(Constants.ArmConstants.pickUpShoulderTick);
    }
}