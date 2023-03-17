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
      camInstance.startClient4("10.31.96.203");
      camInstance.startDSClient();
      
      m_robotContainer = new RobotContainer();

      Drivetrain.zerogyro();

      PortForwarder.add(5800, "10.31.96.33", 5800);
      PortForwarder.add(5800, "10.31.96.44", 5800);

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

      m_robotContainer.elbowAngle = m_robotContainer.getElbowAngle();
      m_robotContainer.shoulderAngle = m_robotContainer.getShoulderAngle();

      OI.ArmElevator.ElbowAngleEntry.setDouble(m_robotContainer.elbowAngle);
      OI.ArmElevator.ShoulderAngleEntry.setDouble(m_robotContainer.shoulderAngle);

      m_robotContainer.currentElevatorPosition = m_robotContainer.getLiftEncoderTick();

      OI.ArmElevator.ElbowPosEntry.setDouble(m_robotContainer.getElbowEncoderTick());
      OI.ArmElevator.ShoulderPosEntry.setDouble(m_robotContainer.getShoulderEncoderTick());
      OI.ArmElevator.ElevatorPosEntry.setDouble(m_robotContainer.currentElevatorPosition);

      OI.ArmElevator.ElbowSetPosEntry.setDouble(RobotContainer.elbowSetPos);
      OI.ArmElevator.ElevatorSetPosEntry.setDouble(RobotContainer.elevatorSetPos);
      OI.ArmElevator.ShoulderSetPosEntry.setDouble(RobotContainer.shoulderSetPos);

      if(m_robotContainer.arm.isResetElbow() != RobotContainer.isElbowSwitchHit){
         m_robotContainer.resetElbowEncoderTick();
      }
      if(m_robotContainer.arm.isResetShoulder() != RobotContainer.isShoulderSwitchHit){
         m_robotContainer.resetShoulderTick();
      }
      RobotContainer.isElbowSwitchHit = m_robotContainer.arm.isResetElbow();
      RobotContainer.isShoulderSwitchHit = m_robotContainer.arm.isResetShoulder();

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
      // RobotContainer.setElbowSetPoint(-7.2);
      
      RobotContainer.drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
      RobotContainer.drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);

      RobotContainer.setElbowSetPoint(0);
      RobotContainer.setElevatorSetPoint(40);
      RobotContainer.setShoulderSetPoint(-3);

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
   if(m_robotContainer.isElevLimitPressed() ==true)
      RobotContainer.setShoulderSetPoint(Constants.ArmConstants.highCubeShoulderTick);
      RobotContainer.setElbowSetPoint(Constants.ArmConstants.highCubeElbowTick);
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
      m_robotContainer.resetElevatorIAccum();
      RobotContainer.setElbowSetPoint(0);
      RobotContainer.setShoulderSetPoint(0);
   
     if(autoCommand != null){
       autoCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      //  if(Math.abs(RobotContainer.LJSY_Arm) < 0.08)
      //     RobotContainer.LJSY_Arm = 0;
      //  RobotContainer.setElevatorSetPoint(RobotContainer.getElevatorSetPoint() + (RobotContainer.LJSY_Arm * -2));
      // if(RobotContainer.getElbowSetPoint() < -7.6){
      //    RobotContainer.setElbowSetPoint(-7.6);
      // }
      // if(RobotContainer.getElbowSetPoint() >0){
      //    RobotContainer.setElbowSetPoint(0);
      // }
      if (OI.XboxController.X2_DPad == 270) {

         m_robotContainer.resetElevatorIAccum();
         RobotContainer.setShoulderSetPoint(Constants.ArmConstants.restShoulderTick);
         RobotContainer.setElbowSetPoint(Constants.ArmConstants.restElbowTick);
         
     }
      //Low Pos Pickup Position
      if(OI.XboxController.X2_DPad == 0)  {
          RobotContainer.setShoulderSetPoint(Constants.ArmConstants.highCubeShoulderTick);
          RobotContainer.setElbowSetPoint(Constants.ArmConstants.highCubeElbowTick);
      }
      if (OI.XboxController.X2_DPad == 180) {
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.lowElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.lowShoulderTick);
      }
      if(OI.XboxController.X2_DPad == 90 && RobotContainer.isCube) {
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.midCubeElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.midCubeShoulderTick);
      }
      else if(OI.XboxController.X2_DPad == 90 && RobotContainer.isCube == false){
            RobotContainer.setElbowSetPoint(Constants.ArmConstants.midConeElbowTick);
            RobotContainer.setShoulderSetPoint(Constants.ArmConstants.midConeShoulderTick);
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