package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Autonomous.Positions.AutoArmToRestPos;

public class AutoBackArmRest extends ParallelCommandGroup{
    Drivetrain drivetrain;

    public AutoBackArmRest(Drivetrain drivetrain) {
        super(
            new DriveBackwardsAuto(drivetrain),
            new AutoArmToRestPos()
        );
    }
}
