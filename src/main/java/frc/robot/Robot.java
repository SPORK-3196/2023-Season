// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.NeutralMode;

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
      NetworkTableInstance cam2Instance = NetworkTableInstance.getDefault();

      camInstance.startClient4("10.31.96.214");
      camInstance.startDSClient();
      cam2Instance.startClient4("10.31.96.4");
      cam2Instance.startDSClient();
      
      m_robotContainer = new RobotContainer();

      RobotContainer.drivetrain.zeroGyro();

      PortForwarder.add(5800, "10.31.96.214", 5800);

      RobotContainer.raspiCam.setDriverMode(false);
      RobotContainer.raspiCam.setPipelineIndex(0);

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
      RobotContainer.R_TPrimary = RobotContainer.primaryController.getRightTriggerAxis();

      RobotContainer.LJSY_Arm = RobotContainer.armController.getLeftY();
      RobotContainer.RJSX_Arm = RobotContainer.armController.getRightX();

      OI.Drivetrain.gyroRate = Drivetrain.getGyroRate();
      OI.Drivetrain.gyroHeading = RobotContainer.getPoseRotation();

      m_robotContainer.elbowAngle = m_robotContainer.getElbowAngle();
      //m_robotContainer.shoulderAngle = m_robotContainer.get

      OI.ArmElevator.ElbowAngleEntry.setDouble(m_robotContainer.elbowAngle);
      OI.ArmElevator.ShoulderAngleEntry.setDouble(m_robotContainer.shoulderAngle);

      // m_robotContainer.currentElevatorPosition = m_robotContainer.getLiftEncoderTick();

      OI.ArmElevator.ElbowPosEntry.setDouble(m_robotContainer.getElbowEncoderTick());
      OI.ArmElevator.ShoulderPosEntry.setDouble(m_robotContainer.getShoulderEncoderTick());
      OI.ArmElevator.ElevatorPosEntry.setDouble(m_robotContainer.currentElevatorPosition);

      OI.ArmElevator.ElbowSetPosEntry.setDouble(RobotContainer.elbowSetPos);
      OI.ArmElevator.ElevatorSetPosEntry.setDouble(RobotContainer.elevatorSetPos);
      OI.ArmElevator.ShoulderSetPosEntry.setDouble(RobotContainer.shoulderSetPos);

      if(m_robotContainer.arm.isResetElbow()){
         System.out.println("Slider reset");
         m_robotContainer.resetElbowEncoderTick();
       }
       if(m_robotContainer.isShoulderLimitPressed()){
         //System.out.println("shoulder reset");
          m_robotContainer.resetShoulderTick();
      }
      if(m_robotContainer.arm.isStopElbow()){
         m_robotContainer.arm.turnElbowOff();
      }
      if(m_robotContainer.arm.isStopShoulder()){
         m_robotContainer.arm.turnShoulderOff();;
      }

      RobotContainer.isElbowSwitchHit = m_robotContainer.arm.isResetElbow();
      RobotContainer.isShoulderSwitchHit = m_robotContainer.arm.isResetShoulder();

      if(RobotContainer.isCube == false)
         OI.Vision.TypeOfPieceEntry.setString("Cone");
      else
         OI.Vision.TypeOfPieceEntry.setString("Cube");
   
      OI.Vision.piCamHasTargets = 
         RobotContainer.hasTargets(
            RobotContainer.pipelineResult(
               RobotContainer.raspiCam));
      
      RobotContainer.piResult = RobotContainer.pipelineResult(RobotContainer.raspiCam);

      if(RobotContainer.piResult != null){
            RobotContainer.piBResult = RobotContainer.pipelineResult(RobotContainer.raspiCam).getBestTarget();
            RobotContainer.piYaw = RobotContainer.getCamYaw(RobotContainer.piBResult);
            RobotContainer.piX = RobotContainer.distanceToVisionPose(RobotContainer.piBResult).getX();
            RobotContainer.piY = RobotContainer.distanceToVisionPose(RobotContainer.piBResult).getY();
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
         OI.Drivetrain.GyroHeadingEntry.setDouble(RobotContainer.getGyroYaw().getDegrees());

         OI.Drivetrain.poseX = RobotContainer.getPoseX();
         OI.Drivetrain.poseY = RobotContainer.getPoseY();
         OI.Drivetrain.PoseXEntry.setDouble(RobotContainer.getDrivetrainOdometry().getX());
         OI.Drivetrain.PoseYEntry.setDouble(RobotContainer.getDrivetrainOdometry().getY());

         OI.Drivetrain.GyroPitchEntry.setDouble(RobotContainer.getGyroPitch());
         OI.Vision.TurretPosEntry.setDouble(m_robotContainer.getTurretPos());
         if(RobotContainer.piBResult != null){
            OI.Vision.RaspiYawEntry.setDouble(RobotContainer.getCamYaw(RobotContainer.piBResult));
            OI.Vision.distanceToPieceXEntry.setDouble(RobotContainer.piX);
            OI.Vision.distanceToPieceYEntry.setDouble(RobotContainer.piY);
         }
         OI.Vision.raspiTargetEntry.setBoolean(OI.Vision.piCamHasTargets);
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
      // RobotContainer.setElbowSetPoint(-7.2);
      
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
      RobotContainer.setElbowSetPoint(0);
      RobotContainer.setShoulderSetPoint(0);
   
     if(autoCommand != null){
       autoCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

      if (OI.XboxController.X2_DPad == 180) {

         RobotContainer.setShoulderSetPoint(Constants.ArmConstants.restShoulderTick);
         RobotContainer.setElbowSetPoint(Constants.ArmConstants.restElbowTick);
     }

      //Low Pos Pickup Position
      if(OI.XboxController.X2_DPad == 0 )  {
          RobotContainer.setShoulderSetPoint(Constants.ArmConstants.highCubeShoulderTick);
          RobotContainer.setElbowSetPoint(Constants.ArmConstants.highCubeElbowTick);
      }
      if (OI.XboxController.X2_DPad == 90 && RobotContainer.isCube) {
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.lowElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.lowShoulderTick);
      }
      if(OI.XboxController.X2_DPad == 90 && RobotContainer.isCube == false){
            RobotContainer.setElbowSetPoint(-.5);
            RobotContainer.setShoulderSetPoint(-.5);
      }
      if(OI.XboxController.X2_DPad == 270 && RobotContainer.isCube) {
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.midCubeElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.midCubeShoulderTick);
      }
      else if(OI.XboxController.X2_DPad == 90 && RobotContainer.isCube == false){
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.midConeElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.midConeShoulderTick);
      }
      if(RobotContainer.getElbowSetPoint() == 0 && RobotContainer.getShoulderSetPoint() == 0){
         
      }


  }

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