package frc.robot.commands.Autonomous;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBackwardsAuto extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    PIDController turnController;
    double turn;
    double time;
    double power;
    public DriveBackwardsAuto(Drivetrain drivetrain, double time, double power){
        this.drivetrain = drivetrain;
        this.time = time;
        this.power= power;
        timer = new Timer();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.zeroGyro();
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
       
        drivetrain.frontLeft.configOpenloopRamp(1);
        drivetrain.frontRight.configOpenloopRamp(1);
        drivetrain.rearLeft.configOpenloopRamp(1);
        drivetrain.rearRight.configOpenloopRamp(1);


        turnController = new PIDController(.03, .0025, 0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        drivetrain.arcadeDrive(power, 0);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.arcadeDrive(0, 0);
    }
    public boolean isFinished(){
        return timer.hasElapsed(time);
    }
}