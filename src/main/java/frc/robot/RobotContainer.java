package frc.robot;

import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.Drivetrain.DriveWithJoyStick;
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
        configureButtonBindings();
        drivetrain.setDefaultCommand(joystickDrive);
        claw.setDefaultCommand(new OpenClaw(claw));
    }


    public void configureButtonBindings() {

    }
}
