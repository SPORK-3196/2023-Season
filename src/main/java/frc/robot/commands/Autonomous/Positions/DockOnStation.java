package frc.robot.commands.Autonomous.Positions;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DockOnStation extends CommandBase {
    Drivetrain drivetrain;
    Timer timer;
    PIDController dockController;
    double power;
    double turn;
    double pitch;
    Timer timer2;
    public DockOnStation(Drivetrain drivetrain){
        this.drivetrain = drivetrain;
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

        dockController = new PIDController(.2, 0, .001);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        pitch = RobotContainer.getGyroPitch();
        power = dockController.calculate(pitch, -2);
        dockController.setTolerance(1);
        if(power > .31){
            power = .31;
        }
        else if(power < -.31){
            power = -.31;
        }
        drivetrain.arcadeDrive(power, 0);
        
    }

    @Override 
    public void end(boolean interrupted){
            drivetrain.arcadeDrive(0, 0);
    }
    public boolean isFinished(){
        return dockController.atSetpoint();
    }
}