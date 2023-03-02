package frc.robot.commands.Autonomous.Cube;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TrackToCube extends CommandBase{
    Drivetrain drivetrain;
    PIDController controller;
    double kp, kd, ki;

    public TrackToCube(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        
        kp = .05;
        kd = 0;
        ki = 0;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        controller = new PIDController(kp, ki, kd);
        controller.setTolerance(.5);
        
        RobotContainer.raspiCam.setPipelineIndex(1);
    }

    @Override
    public void execute(){
        double gyroAngle = RobotContainer.getCamYaw(RobotContainer.rasbResult);
        double speed = controller.calculate(gyroAngle);

        drivetrain.arcadeDriveAI(0, speed);
    }
    
    @Override
    public void end(boolean isFinished){
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public boolean isFinished(){
        return controller.atSetpoint();
    }
}
