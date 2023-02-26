package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
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
        kp = .08;
        kd= .0075;
        ki = 0;
    }
    @Override
    public void initialize(){
        drivetrain.zeroGyro();
        controller = new PIDController(kp, ki, kd);
        controller.setTolerance(.5);
    }
    
    @Override
    public void execute(){ 
        heading = Drivetrain.getGyroHeading();
        speed = controller.calculate(heading, -90);
        
        if(speed < 0)
            speed = Math.max(speed, -0.4);
        
        else 
            speed = Math.min(speed, .4);

        drivetrain.arcadeDrive(0, speed);      
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
    }
    public boolean isFinished(){
        return controller.atSetpoint();
        
    } 
}
