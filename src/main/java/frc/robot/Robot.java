// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
   private Command autoCommand;
  private RobotContainer m_robotContainer;


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
      NetworkTableInstance camInstance = NetworkTableInstance.getDefault();
      camInstance.startClient4("10.31.96.16");
      camInstance.startDSClient();
      
      m_robotContainer = new RobotContainer();

      Drivetrain.zerogyro();

      PortForwarder.add(5800, "10.31.96.16", 5800);
      PortForwarder.add(5800, "10.31.96.44", 5800);

      // HttpCamera aprilCameraVideoFeed = new HttpCamera("G_S", "10.31.96.16:5800");
      // HttpCamera raspberryPiVideoFeed = new HttpCamera("Raspi Cam", "10.31.96.44:5800");

      // OI.Vision.Vision_TAB.add("Global Shutter Feed", aprilCameraVideoFeed);
      // OI.Vision.Vision_TAB.add("Raspi Cam Feed", raspberryPiVideoFeed);

      RobotContainer.aprilTagCam.setDriverMode(true);
      RobotContainer.aprilTagCam.setPipelineIndex(1);
      RobotContainer.raspiCam.setDriverMode(true);

      PhotonCamera.setVersionCheckEnabled(false);
      
      Shuffleboard.getTab("Autonomous Controls")
      .add(RobotContainer.autoChooser);

      DriverStation.silenceJoystickConnectionWarning(true);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
      RobotContainer.LJSX_Primary = RobotContainer.primaryController.getLeftX();
      RobotContainer.LJSY_Primary = RobotContainer.primaryController.getLeftY();
      RobotContainer.LJSY_Arm = RobotContainer.armController.getLeftY();
      RobotContainer.RJSX_Arm = RobotContainer.armController.getRightX();

      OI.Drivetrain.gyroRate = Drivetrain.getGyroRate();
      OI.Drivetrain.gyroHeading = RobotContainer.getPoseRotation();

      RobotContainer.elevatorSetPos = RobotContainer.getElevatorSetPoint();
      RobotContainer.shoulderSetPos = RobotContainer.getShoulderSetPoint();
      RobotContainer.elbowSetPos = RobotContainer.getElbowSetPoint();

      m_robotContainer.currentElevatorPosition = m_robotContainer.getLiftEncoderTick();

      OI.ArmElevator.ElbowPosEntry.setDouble(m_robotContainer.getElbowEncoderTick());
      OI.ArmElevator.ShoulderPosEntry.setDouble(m_robotContainer.getShoulderEncoderTick());
      OI.ArmElevator.ElevatorPosEntry.setDouble(m_robotContainer.currentElevatorPosition);

      if(RobotContainer.getPickUp() == false){

         //Update Elbow and Shoulder Position based off elevator position between rest and low position

         if(m_robotContainer.currentElevatorPosition > Constants.LiftConstants.liftRestTick && m_robotContainer.currentElevatorPosition < Constants.LiftConstants.liftBottomTick){
            RobotContainer.currentShoulderSetPos = Constants.ArmConstants.restShoulderTick + 
               (((m_robotContainer.currentElevatorPosition-Constants.LiftConstants.liftRestTick)/
                  (Constants.LiftConstants.liftBottomTick- Constants.LiftConstants.liftRestTick)) * 
                  (Constants.ArmConstants.lowShoulderTick - Constants.ArmConstants.restShoulderTick));  
            RobotContainer.currentElbowSetPos =  Constants.ArmConstants.restElbowTick +
               (((m_robotContainer.currentElevatorPosition - Constants.LiftConstants.liftRestTick) /
               (Constants.LiftConstants.liftBottomTick - Constants.LiftConstants.liftRestTick)) *
               (Constants.ArmConstants.lowElbowTick - Constants.ArmConstants.restElbowTick));
         }
         //Update Elbow and Shoulder Position based off elevator position between low and mid position

         else if(m_robotContainer.currentElevatorPosition > Constants.LiftConstants.liftBottomTick && m_robotContainer.currentElevatorPosition < Constants.LiftConstants.liftMidTick){
            RobotContainer.currentShoulderSetPos = Constants.ArmConstants.lowShoulderTick + 
               (((m_robotContainer.currentElevatorPosition-Constants.LiftConstants.liftBottomTick)/
                  (Constants.LiftConstants.liftMidTick- Constants.LiftConstants.liftBottomTick)) * 
                  (Constants.ArmConstants.midShoulderTick - Constants.ArmConstants.lowShoulderTick));  
            RobotContainer.currentElbowSetPos =  Constants.ArmConstants.lowElbowTick +
               (((m_robotContainer.currentElevatorPosition - Constants.LiftConstants.liftBottomTick) /
               (Constants.LiftConstants.liftMidTick - Constants.LiftConstants.liftBottomTick)) *
               (Constants.ArmConstants.midElbowTick - Constants.ArmConstants.lowElbowTick));
         }
         //Update Elbow and Shoulder Position based off elevator position between mid and high position

         else if(m_robotContainer.currentElevatorPosition > Constants.LiftConstants.liftMidTick && m_robotContainer.currentElevatorPosition < Constants.LiftConstants.liftTopTick){
            RobotContainer.currentShoulderSetPos = Constants.ArmConstants.midShoulderTick + 
               (((m_robotContainer.currentElevatorPosition-Constants.LiftConstants.liftMidTick)/
                  (Constants.LiftConstants.liftTopTick- Constants.LiftConstants.liftMidTick)) * 
                  (Constants.ArmConstants.highShoulderTick - Constants.ArmConstants.midShoulderTick));  
            RobotContainer.currentElbowSetPos =  Constants.ArmConstants.midElbowTick +
               (((m_robotContainer.currentElevatorPosition - Constants.LiftConstants.liftMidTick) /
               (Constants.LiftConstants.liftTopTick - Constants.LiftConstants.liftMidTick)) *
               (Constants.ArmConstants.highElbowTick - Constants.ArmConstants.midElbowTick));
         }
      }
      else{
         //Update Elbow and Shoulder Position based off elevator position between rest and station position

         if(m_robotContainer.currentElevatorPosition < Constants.LiftConstants.liftStationTick && m_robotContainer.currentElevatorPosition > Constants.LiftConstants.liftRestTick){
            RobotContainer.currentShoulderSetPos = Constants.ArmConstants.restShoulderTick + 
               (((m_robotContainer.currentElevatorPosition-Constants.LiftConstants.liftRestTick)/
                  (Constants.LiftConstants.liftStationTick- Constants.LiftConstants.liftRestTick)) * 
                  (Constants.ArmConstants.pickUpStationShoulderTick - Constants.ArmConstants.restShoulderTick));  
            RobotContainer.currentElbowSetPos =  Constants.ArmConstants.restElbowTick +
               (((m_robotContainer.currentElevatorPosition - Constants.LiftConstants.liftRestTick) /
               (Constants.LiftConstants.liftStationTick - Constants.LiftConstants.liftRestTick)) *
               (Constants.ArmConstants.pickUpStationElbowTick - Constants.ArmConstants.restElbowTick));
         }
         //Update Elbow and Shoulder Position based off elevator position between rest and station position
         else if(m_robotContainer.currentElevatorPosition < Constants.LiftConstants.liftPickUpTick && m_robotContainer.currentElevatorPosition > Constants.LiftConstants.liftStationTick){
            RobotContainer.currentShoulderSetPos = Constants.ArmConstants.pickUpStationShoulderTick + 
               (((m_robotContainer.currentElevatorPosition-Constants.LiftConstants.liftStationTick)/
                  (Constants.LiftConstants.liftPickUpTick - Constants.LiftConstants.liftStationTick)) * 
                  (Constants.ArmConstants.pickUpShoulderTick - Constants.ArmConstants.pickUpStationShoulderTick));  
            RobotContainer.currentElbowSetPos =  Constants.ArmConstants.pickUpStationElbowTick +
               (((m_robotContainer.currentElevatorPosition - Constants.LiftConstants.liftStationTick) /
               (Constants.LiftConstants.liftPickUpTick - Constants.LiftConstants.liftStationTick)) *
               (Constants.ArmConstants.pickUpElbowTick - Constants.ArmConstants.pickUpStationElbowTick));
         }
      }

      if(m_robotContainer.lift.isResetLift() == true){
         m_robotContainer.resetLiftEncoderTick();
      }
      if(m_robotContainer.arm.isResetElbow() == true){
         m_robotContainer.resetElbowEncoderTick();
      }
      if(m_robotContainer.arm.isResetShoulder() == true){
         m_robotContainer.resetShoulderTick();
      }

      if(RobotContainer.isCube == false)
         OI.Vision.TypeOfPieceEntry.setString("Cone");
      else
         OI.Vision.TypeOfPieceEntry.setString("Cube");
      
      OI.Vision.aprilCamHasTargets = 
         RobotContainer.hasTargets(
            RobotContainer.pipelineResult(
               RobotContainer.aprilTagCam));
      
      RobotContainer.result = RobotContainer.pipelineResult(RobotContainer.aprilTagCam);
      if(RobotContainer.hasTargets(RobotContainer.result)){
         RobotContainer.bResult = RobotContainer.result.getBestTarget();
         RobotContainer.rasbResult = RobotContainer.pipelineResult(RobotContainer.raspiCam).getBestTarget();
         RobotContainer.aprilYaw = RobotContainer.getCamYaw(RobotContainer.bResult);
         RobotContainer.aprilX = RobotContainer.distanceToVisionPose(RobotContainer.bResult).getX();
         RobotContainer.aprilY = RobotContainer.distanceToVisionPose(RobotContainer.bResult).getY();
         }

      if(RobotContainer.primaryController.isConnected()) {

         OI.XboxController.X1_AButton = RobotContainer.primaryController.getAButton();
         OI.XboxController.X1_BButton = RobotContainer.primaryController.getBButton();
         OI.XboxController.X1_XButton = RobotContainer.primaryController.getXButton();
         OI.XboxController.X1_YButton = RobotContainer.primaryController.getYButton();

         OI.XboxController.X1_LJX = RobotContainer.primaryController.getLeftX();
         OI.XboxController.X1_LJY = RobotContainer.primaryController.getLeftY();
      }
      if(RobotContainer.armController.isConnected()){
         OI.XboxController.X2_DPad = RobotContainer.armController.getPOV();

      }

      if(!DriverStation.isFMSAttached()){
         OI.XboxController.X1_AButtonEntry.setBoolean(OI.XboxController.X1_AButton);
         OI.XboxController.X1_BButtonEntry.setBoolean(OI.XboxController.X1_BButton);
         OI.XboxController.X1_XButtonEntry.setBoolean(OI.XboxController.X1_XButton);
         OI.XboxController.X1_YButtonEntry.setBoolean(OI.XboxController.X1_YButton);
         
         OI.XboxController.X1_LJX_Entry.setDouble(OI.XboxController.X1_LJX);
         OI.XboxController.X1_LJY_Entry.setDouble(OI.XboxController.X1_LJY);

         OI.XboxController.X2_DPadEntry.setDouble(OI.XboxController.X2_DPad);

         OI.Drivetrain.GyroRateEntry.setDouble(OI.Drivetrain.gyroRate);
         OI.Drivetrain.GyroHeadingEntry.setDouble(OI.Drivetrain.gyroHeading);

         OI.Drivetrain.poseX = RobotContainer.getPoseX();
         OI.Drivetrain.poseY = RobotContainer.getPoseY();
         OI.Drivetrain.PoseXEntry.setDouble(OI.Drivetrain.poseX);
         OI.Drivetrain.PoseYEntry.setDouble(OI.Drivetrain.poseY);
         if(RobotContainer.bResult != null)  
            OI.Vision.MicroYawEntry.setDouble(RobotContainer.getCamYaw(RobotContainer.bResult));
            OI.Vision.distanceToTagXEntry.setDouble(RobotContainer.aprilX);
            OI.Vision.distanceToTagYEntry.setDouble(RobotContainer.aprilY);
         OI.Vision.LimelightTargetsEntry.setBoolean(OI.Vision.aprilCamHasTargets);
      }
   
      CommandScheduler.getInstance().run();  
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
      RobotContainer.elevatorSetPos = Constants.LiftConstants.liftRestTick;
      RobotContainer.elbowSetPos = Constants.ArmConstants.restElbowTick;
      RobotContainer.shoulderSetPos = Constants.ArmConstants.restShoulderTick;
      
      RobotContainer.drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);

      RobotContainer.drivetrain.resetOdometry();
     autoCommand = m_robotContainer.getSelected();

     if(autoCommand != null)
     {
         autoCommand.schedule();
     }

     //m_Timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   /*  rightGroup.setInverted(true);
    if(m_Timer.get()< 1.5){
    differentialDrive.arcadeDrive(0.1,0,false);
    
    } else {
      differentialDrive.stopMotor();
    }*/
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
      RobotContainer.drivetrain.resetEncoders();
      RobotContainer.drivetrain.resetOdometry();
   
     if(autoCommand != null){
       autoCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}