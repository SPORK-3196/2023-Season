// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;

import static frc.robot.Variables.XboxController.X1_AButton;
import static frc.robot.Variables.XboxController.X1_AButtonEntry;
import static frc.robot.Variables.XboxController.X1_BButton;
import static frc.robot.Variables.XboxController.X1_BButtonEntry;
import static frc.robot.Variables.XboxController.X1_LB;
import static frc.robot.Variables.XboxController.X1_LB_Entry;
import static frc.robot.Variables.XboxController.X1_LJX;
import static frc.robot.Variables.XboxController.X1_LJX_Entry;
import static frc.robot.Variables.XboxController.X1_LJY;
import static frc.robot.Variables.XboxController.X1_LJY_Entry;
import static frc.robot.Variables.XboxController.X1_LTValue;
import static frc.robot.Variables.XboxController.X1_LT_Entry;
import static frc.robot.Variables.XboxController.X1_RB;
import static frc.robot.Variables.XboxController.X1_RB_Entry;
import static frc.robot.Variables.XboxController.X1_RJX;
import static frc.robot.Variables.XboxController.X1_RJX_Entry;
import static frc.robot.Variables.XboxController.X1_RJY;
import static frc.robot.Variables.XboxController.X1_RJY_Entry;
import static frc.robot.Variables.XboxController.X1_RTValue;
import static frc.robot.Variables.XboxController.X1_RT_Entry;
import static frc.robot.Variables.XboxController.X1_XButton;
import static frc.robot.Variables.XboxController.X1_XButtonEntry;
import static frc.robot.Variables.XboxController.X1_YButton;
import static frc.robot.Variables.XboxController.X1_YButtonEntry;
import static frc.robot.Variables.XboxController.X2_AButton;
import static frc.robot.Variables.XboxController.X2_AButtonEntry;
import static frc.robot.Variables.XboxController.X2_BButton;
import static frc.robot.Variables.XboxController.X2_BButtonEntry;
import static frc.robot.Variables.XboxController.X2_LB;
import static frc.robot.Variables.XboxController.X2_LB_Entry;
import static frc.robot.Variables.XboxController.X2_LJX;
import static frc.robot.Variables.XboxController.X2_LJX_Entry;
import static frc.robot.Variables.XboxController.X2_LJY;
import static frc.robot.Variables.XboxController.X2_LJY_Entry;
import static frc.robot.Variables.XboxController.X2_LTValue;
import static frc.robot.Variables.XboxController.X2_LT_Entry;
import static frc.robot.Variables.XboxController.X2_RB;
import static frc.robot.Variables.XboxController.X2_RB_Entry;
import static frc.robot.Variables.XboxController.X2_RJX;
import static frc.robot.Variables.XboxController.X2_RJX_Entry;
import static frc.robot.Variables.XboxController.X2_RJY;
import static frc.robot.Variables.XboxController.X2_RJY_Entry;
import static frc.robot.Variables.XboxController.X2_RTValue;
import static frc.robot.Variables.XboxController.X2_RT_Entry;
import static frc.robot.Variables.XboxController.X2_XButton;
import static frc.robot.Variables.XboxController.X2_XButtonEntry;
import static frc.robot.Variables.XboxController.X2_YButton;
import static frc.robot.Variables.XboxController.X2_YButtonEntry;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


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
  
  public static JoystickButton X1J_A =  new JoystickButton(primaryController, XboxController.Button.kA.value);
  public static JoystickButton X1J_B =  new JoystickButton(primaryController, XboxController.Button.kB.value);
  public static JoystickButton X1J_X =  new JoystickButton(primaryController, XboxController.Button.kX.value);
  public static JoystickButton X1J_Y =  new JoystickButton(primaryController, XboxController.Button.kY.value);
  public static JoystickButton X1J_LB = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
  public static JoystickButton X1J_RB = new JoystickButton(primaryController, XboxController.Button.kRightBumper.value);
  public static JoystickButton X1J_LS = new JoystickButton(primaryController, XboxController.Button.kLeftStick.value);
  public static JoystickButton X1J_RS = new JoystickButton(primaryController, XboxController.Button.kRightStick.value);
  
  public static JoystickButton X2J_A =  new JoystickButton(armController, XboxController.Button.kA.value);
  public static JoystickButton X2J_B =  new JoystickButton(armController, XboxController.Button.kB.value);
  public static JoystickButton X2J_X =  new JoystickButton(armController, XboxController.Button.kX.value);
  public static JoystickButton X2J_Y =  new JoystickButton(armController, XboxController.Button.kY.value);
  public static JoystickButton X2J_LB = new JoystickButton(armController, XboxController.Button.kLeftBumper.value);
  public static JoystickButton X2J_RB = new JoystickButton(armController, XboxController.Button.kRightBumper.value);
  public static JoystickButton X2J_LS = new JoystickButton(armController, XboxController.Button.kLeftStick.value);
  public static JoystickButton X2J_RS = new JoystickButton(armController, XboxController.Button.kRightStick.value);

  public static SlewRateLimiter filter = new SlewRateLimiter(.5);
  public static Object liftMotor; 
  public static Lift lift;
  public static Arm arm;
  
  public static double dPad = armController.getPOV();
  public static JoystickButton B_Arm = new JoystickButton(armController, XboxController.Button.kB.value);
  public static JoystickButton A_Arm = new JoystickButton(armController, XboxController.Button.kA.value);
  public static JoystickButton X_Arm = new JoystickButton(armController, XboxController.Button.kX.value);
  public static JoystickButton Y_Arm = new JoystickButton(armController, XboxController.Button.kY.value);
  WPI_PigeonIMU gyroscope = new WPI_PigeonIMU(0);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Lift lift;
      m_robotContainer = new RobotContainer();
      CommandScheduler.getInstance().run();
      Shuffleboard.getTab("gyroscope").add(gyroscope);
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
    Variables.Drivetrain.gyroRate = Drivetrain.getGyroRate();
    Variables.Drivetrain.gyroHeading = Drivetrain.getGyroHeading();
      if(primaryController.isConnected())
      {
        X1_RTValue = primaryController.getRightTriggerAxis();
        X1_LTValue = primaryController.getLeftTriggerAxis();
        
        X1_RB = primaryController.getRightBumper();
        X1_LB = primaryController.getLeftBumper();
  
        X1_LJX = primaryController.getLeftX();
        X1_LJY = primaryController.getLeftY();
        X1_RJX = primaryController.getRightX();
        X1_RJY = primaryController.getRightY();      
        
        X1_XButton = primaryController.getXButton();
        X1_YButton = primaryController.getYButton();
        X1_AButton = primaryController.getAButton();
        X1_BButton = primaryController.getBButton(); 
      }
      if(!DriverStation.isFMSAttached());{
        X1_RT_Entry.setDouble(X1_RTValue);
        X1_LT_Entry.setDouble(X1_LTValue);

        X1_RB_Entry.setBoolean(X1_RB);
        X1_LB_Entry.setBoolean(X1_LB);

        X1_LJX_Entry.setDouble(X1_LJX);
        X1_LJY_Entry.setDouble(X1_LJY);
        X1_RJX_Entry.setDouble(X1_RJX);
        X1_RJY_Entry.setDouble(X1_RJY);

        X1_XButtonEntry.setBoolean(X1_XButton);
        X1_YButtonEntry.setBoolean(X1_YButton);
        X1_AButtonEntry.setBoolean(X1_AButton);
        X1_BButtonEntry.setBoolean(X1_BButton);
      }
    

      if(armController.isConnected()){

        X2_RTValue = primaryController.getRightTriggerAxis();
        X2_LTValue = primaryController.getLeftTriggerAxis();
        
        X2_RB = primaryController.getRightBumper();
        X2_LB = primaryController.getLeftBumper();
  
        X2_LJX = primaryController.getLeftX();
        X2_LJY = primaryController.getLeftY();
        X2_RJX = primaryController.getRightX();
        X2_RJY = primaryController.getRightY();      
        
        X2_XButton = primaryController.getXButton();
        X2_YButton = primaryController.getYButton();
        X2_AButton = primaryController.getAButton();
        X2_BButton = primaryController.getBButton(); 

      if(!DriverStation.isFMSAttached()){
        X2_RT_Entry.setDouble(X1_RTValue);
        X2_LT_Entry.setDouble(X1_LTValue);

        X2_RB_Entry.setBoolean(X1_RB);
        X2_LB_Entry.setBoolean(X1_LB);

        X2_LJX_Entry.setDouble(X1_LJX);
        X2_LJY_Entry.setDouble(X1_LJY);
        X2_RJX_Entry.setDouble(X1_RJX);
        X2_RJY_Entry.setDouble(X1_RJY);

        X2_XButtonEntry.setBoolean(X1_XButton);
        X2_YButtonEntry.setBoolean(X1_YButton);
        X2_AButtonEntry.setBoolean(X1_AButton);
        X2_BButtonEntry.setBoolean(X1_BButton);
      }
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
