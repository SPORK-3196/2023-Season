// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lift;
import pabeles.concurrency.ConcurrencyOps.Reset;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer m_robotContainer;

  public static XboxController primaryController = new XboxController(0);
  public static XboxController armController = new XboxController(1);

  public static double LJSX_Primary = primaryController.getLeftX();
  public static double LJSY_Primary = primaryController.getLeftY();
  public static SlewRateLimiter filter = new SlewRateLimiter(.5);
  public static Object liftMotor;
  public static Lift lift;
  public static Arm arm;
  private final Timer m_Timer = new Timer();
  public WPI_TalonFX rearLeft = new WPI_TalonFX(DrivetrainConstants.rearLeftPort);
  public WPI_TalonFX frontLeft = new WPI_TalonFX(DrivetrainConstants.frontLeftPort);
  public WPI_TalonFX rearRight = new WPI_TalonFX(DrivetrainConstants.rearRightPort);
  public WPI_TalonFX frontRight = new WPI_TalonFX(DrivetrainConstants.frontRightPort);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(rearLeft, frontLeft);
  private MotorControllerGroup rightGroup = new MotorControllerGroup(rearRight, frontRight);
  public DifferentialDrive differentialDrive = new DifferentialDrive(leftGroup, rightGroup);

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Lift lift;
      m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
     if(autoCommand != null)
     {
        autoCommand.schedule();
     }
     m_Timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   /*  rightGroup.setInverted(true);
    if(m_Timer.get()< 1.5){
    differentialDrive.arcadeDrive(0.01,0,false);
    
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
