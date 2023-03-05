package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DistanceDrive extends CommandBase{
    Drivetrain drivetrain;
    double time;
    double speed;
    double pos;
    Timer timer = new Timer();
    PIDController controller;
    PIDController turnController;
    public DistanceDrive(Drivetrain drivetrain, double time){
        this.drivetrain = drivetrain;
        this.time = time;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.resetEncoders();
        drivetrain.zeroGyro();
        timer.reset();
        timer.start();
        controller = new PIDController(.01, .005, .01);
        turnController = new PIDController(.0075, 0, 0);
    } 

    @Override
    public void execute(){
        pos = drivetrain.sensorToMeters(drivetrain.rearLeft.getSelectedSensorPosition());
        speed = controller.calculate(pos, .5);
        drivetrain.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.zeroGyro();
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= time;
    }
}
