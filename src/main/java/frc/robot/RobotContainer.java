package frc.robot;

import edu.wpi.first.wpilibj.XboxController; 

import frc.robot.commands.DriveWithJoyStick;
import frc.robot.commands.TurretDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private Claw claw = new Claw();
    private Drivetrain drivetrain = new Drivetrain();
    private Arm arm = new Arm(); 
    private Elevator elevator = new Elevator();
    private Turret turret = new Turret();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);

    public RobotContainer(){
        turret.setDefaultCommand(new TurretDrive(turret));
        configureButtonBindings();
    }

    public void configureButtonBindings() {

    }
}
