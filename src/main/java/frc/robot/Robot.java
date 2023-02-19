// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
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
      camInstance.startClient4("10.31.96.50");
      camInstance.startDSClient();
      
      m_robotContainer = new RobotContainer();
      Drivetrain.zerogyro();

      PortForwarder.add(5800, "10.31.96.11", 5800);
      PortForwarder.add(5800, "10.31.96.50", 5800);

      RobotContainer.aprilTagCam.setDriverMode(false);
      RobotContainer.aprilTagCam.setPipelineIndex(1);
      PhotonCamera.setVersionCheckEnabled(false);
      
      Shuffleboard.getTab("Autonomous Controls")
      .add(RobotContainer.autoChooser);
      
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
      OI.Drivetrain.gyroRate = Drivetrain.getGyroRate();
      OI.Drivetrain.gyroHeading = Drivetrain.getGyroHeading();
      
      
      RobotContainer.result = RobotContainer.aprilTagCam.getLatestResult();
      if(RobotContainer.hasTargets(RobotContainer.pipelineResult(RobotContainer.aprilTagCam)))
         RobotContainer.bResult = RobotContainer.result.getBestTarget();
         RobotContainer.aprilYaw = RobotContainer.getCamYaw(RobotContainer.bResult);

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

         OI.Vision.MicroYawEntry.setDouble(RobotContainer.getCamYaw(RobotContainer.bResult));
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
     autoCommand = m_robotContainer.getSelected();
     System.out.println("Got here-autonomous init");
     System.out.println("Got here: " + autoCommand);

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
