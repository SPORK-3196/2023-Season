package frc.robot.commands.Autonomous.Positions;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveBackOnChargeStation extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    PIDController turnController;
    double power;
    double turn;
    double pitch;
    boolean isOnRamp = false;
    boolean endCommand = false;
    public DriveBackOnChargeStation(Drivetrain drivetrain, double power){
        this.drivetrain = drivetrain;
        this.power = power;
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
        
        drivetrain.frontLeft.configOpenloopRamp(1.5);
        drivetrain.frontRight.configOpenloopRamp(1.5);
        drivetrain.rearLeft.configOpenloopRamp(1.5);
        drivetrain.rearRight.configOpenloopRamp(1.5);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        pitch = RobotContainer.getGyroPitch();
        if(pitch > 13)
            isOnRamp = true;
        
        if(isOnRamp && pitch <11.9)
            endCommand = true;

        drivetrain.arcadeDrive(power, 0);
        if(RobotContainer.getDrivetrainOdometry().getX() > 4){
            endCommand = true;
        }
    }

    @Override 
    public void end(boolean interrupted){
        drivetrain.arcadeDrive(0, 0);
    }
    public boolean isFinished(){
        return endCommand;
    }
}