package frc.robot;

import frc.robot.commands.DriveWithJoyStick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Turret;

public class RobotContainer {
    private Claw claw = new Claw();
    private Drivetrain drivetrain = new Drivetrain();
    private Arm arm = new Arm(); 
    private Lift lift= new Lift();
    private Turret turret = new Turret();

    private DriveWithJoyStick joystickDrive = new DriveWithJoyStick(drivetrain);
    public RobotContainer(){
        configureButtonBindings();
    }

    public void configureButtonBindings() {

    }
}
