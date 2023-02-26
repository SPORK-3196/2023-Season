package frc.robot.commands.Drivetrain;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.OI;

public class DistanceToVisionTarget extends CommandBase {
    Drivetrain drivetrain;
    PIDController distanceController;
    PIDController turnController; 
    double speed;
    double rotation;

    public DistanceToVisionTarget(Drivetrain drivetrain){
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
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

        distanceController = new PIDController(1, 0, 0);
        turnController = new PIDController(0.05, 0, 0);

    }
    @Override
    public void execute(){
        if(RobotContainer.hasTargets(RobotContainer.result)){
            speed = distanceController.calculate(RobotContainer.distanceToVisionXTargetMeters(), 1.5);
            rotation = turnController.calculate(RobotContainer.getCamYaw(RobotContainer.bResult), 0);
        }

        OI.Vision.PIDOutput = rotation;
        OI.Vision.PIDoutputEntry.setDouble(OI.Vision.PIDOutput);
        OI.Vision.error = turnController.getPositionError();
        OI.Vision.PIDError.setDouble(OI.Vision.error);

        if(speed < 0)
            speed = Math.max(speed, -0.5);    
        else 
            speed = Math.min(speed, .5);

        if(rotation < 0)
            rotation = Math.max(rotation, -0.4);    
        else 
            rotation = Math.min(rotation, .4);
        
        drivetrain.arcadeDrive(speed, rotation);
        
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
        return Math.abs(distanceController.getPositionError()) < 0.05; 
    }
}
