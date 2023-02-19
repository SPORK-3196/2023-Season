package frc.robot.commands.Drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class TracktoTag extends CommandBase {
    Drivetrain drivetrain;
    PIDController controller;
    double rotPower;
    double yaw;

    public TracktoTag(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
        
        controller = new PIDController(.008, 0, 0);

    }

    @Override
    public void execute(){
        
        if(RobotContainer.hasTargets(RobotContainer.result))
            yaw = RobotContainer.getCamYaw(RobotContainer.camResult(RobotContainer.aprilTagCam));
            rotPower = controller.calculate(yaw, -1);

        if (yaw > 1.0) {
            rotPower -= 0.02;
        }
        else if (yaw < 1.0) {
            rotPower += 0.02;
        }
        drivetrain.arcadeDrive(0, rotPower);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
