package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class TurnSetDegrees extends CommandBase  {
    Drivetrain drivetrain;
    public static double heading;
    public double speed;
    public PIDController controller;
    double error;
    double degree;
    Timer timer = new Timer();
    
    public TurnSetDegrees(Drivetrain drivetrain, double degree){
        this.drivetrain = drivetrain;
        this.degree = degree;

        addRequirements(drivetrain);
    }
    @Override
    public void initialize(){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.zeroGyro();
        Drivetrain.gyroscope.setYaw(0);
        controller = new PIDController(.054, 0, .0085);
        timer.reset();
        timer.start();

    }
    @Override
    public void execute(){ 
        heading = Drivetrain.getGyroHeading();
        speed = -controller.calculate(heading, degree);
        if(speed > .45)
            speed =.45;
        else if(speed < -.45)
            speed = -.45;
        error = controller.getPositionError();
        OI.Drivetrain.Turn180Error.setDouble(error);
        OI.Drivetrain.Turn180Heading.setDouble(heading);
        controller.setTolerance(1);
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
        return timer.hasElapsed(2);
        
    } 
}