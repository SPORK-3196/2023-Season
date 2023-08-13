package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Autonomous.Positions.AutoArmToRestPos;

public class AutoBackArmRest extends ParallelCommandGroup{
    Drivetrain drivetrain;
    double power;
    double time;
    public AutoBackArmRest(Drivetrain drivetrain, double power, double time) {
        super(
            new AutoArmToRestPos(),
            new DriveBackwardsAuto(drivetrain, time, power)
        );
    }
}