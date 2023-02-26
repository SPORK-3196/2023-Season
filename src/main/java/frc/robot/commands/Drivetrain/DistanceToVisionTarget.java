package frc.robot.commands.Drivetrain;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.OI;

public class DistanceToVisionTarget extends CommandBase {
    Drivetrain drivetrain;
    PIDController controller;
    double kp, kd, ki;
    double speed;

    public DistanceToVisionTarget(Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
        
        kp = 5;
        kd= 1;
        ki = 0;
                

    }

    @Override
    public void initialize(){
        drivetrain.rearLeft.configOpenloopRamp(1);
        drivetrain.frontLeft.configOpenloopRamp(1);
        drivetrain.rearRight.configOpenloopRamp(1);
        drivetrain.frontRight.configOpenloopRamp(1);
        
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);

        controller = new PIDController(kp, ki, kd);

    }
    @Override
    public void execute(){
        if(RobotContainer.hasTargets(RobotContainer.result))
            speed = controller.calculate(RobotContainer.distanceToVisionTargetMeters(), 1.5);       
        OI.Vision.PIDOutput = speed;
        OI.Vision.PIDoutputEntry.setDouble(OI.Vision.PIDOutput);
        OI.Vision.error = controller.getPositionError();
        OI.Vision.PIDError.setDouble(OI.Vision.error);
        if(speed < 0)
            speed = Math.max(speed, -0.5);
        
        else 
            speed = Math.min(speed, .5);
        
        drivetrain.arcadeDrive(speed, 0);
    }
    @Override 
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);
    }
    @Override
    public boolean isFinished(){
        return Math.abs(controller.getPositionError()) < 0.05; 
    }
}
