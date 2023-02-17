package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Turn45Degrees extends CommandBase  {
    Drivetrain drivetrain;
    double kP = .5;
    public static double heading;
    public double speed;
    public double deadband = .08;
    public PIDController controller;
    private double time;
    Timer timer = new Timer();
    
    public Turn45Degrees(Drivetrain drivetrain, double time){
        this.drivetrain = drivetrain;
        this.time=time;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        drivetrain.zeroGyro();
        controller = new PIDController(.006, .005, 0);
        timer.reset();
        timer.start();

    }
    @Override
    public void execute(){ 
        heading = Drivetrain.getGyroHeading();
        speed = controller.calculate(heading, 90);
        drivetrain.arcadeDrive(0, speed);
    }
    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
        System.out.println(heading);
    }
    public boolean isFinished(){
        return timer.get() >= time;
        
    } 
}
