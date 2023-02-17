package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn45Degrees extends CommandBase  {
    Drivetrain drivetrain;
    double kP = .5;
    public static double heading;
    public double speed;
    public double deadband = .08;
    
    public Turn45Degrees(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        drivetrain.zeroGyro();
        drivetrain.differentialDrive.setDeadband(.08);
    }
    @Override
    public void execute(){ 
        PIDController controller = new PIDController(.018, 0, 0);
        heading = Drivetrain.getGyroHeading();
        speed = controller.calculate(heading, 45);
        drivetrain.arcadeDrive(0, speed);
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.arcadeDrive(0, 0);
        drivetrain.zeroGyro();
    }
    public boolean isFinished(){
        return (heading >= 45 && speed < deadband);
    
    } 
}
