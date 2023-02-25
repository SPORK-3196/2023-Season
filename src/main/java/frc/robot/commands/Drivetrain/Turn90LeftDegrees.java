package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn90LeftDegrees extends CommandBase  {
    Drivetrain drivetrain;
    public static double heading;
    public double speed;
    public PIDController controller;
    double kp, kd, ki;
    
    public Turn90LeftDegrees(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        kp = .015;
        kd= 0;
        ki = 0;
    }
    @Override
    public void initialize(){
        drivetrain.zeroGyro();
        controller = new PIDController(kp, ki, kd);
        controller.setTolerance(1);
    }
    @Override
    public void execute(){ 
        heading = Drivetrain.getGyroHeading();
        speed = controller.calculate(heading, -90);
        
        if(speed < 0)
            speed = Math.max(speed, -0.3);
        
        else 
            speed = Math.min(speed, .3);

        drivetrain.arcadeDrive(0, speed);      
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    }
    public boolean isFinished(){
        return controller.atSetpoint();
        
    } 
}
