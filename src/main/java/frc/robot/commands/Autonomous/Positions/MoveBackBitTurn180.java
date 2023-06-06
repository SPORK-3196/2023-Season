package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.DriveBackwardsAuto;
import frc.robot.commands.Drivetrain.TurnSetDegrees;
import frc.robot.subsystems.Drivetrain;

public class MoveBackBitTurn180 extends SequentialCommandGroup{
    Drivetrain drivetrain;

    public MoveBackBitTurn180(Drivetrain drivetrain){
        super(
            new DriveBackwardsAuto(drivetrain, .5, .3),
            new TurnSetDegrees(drivetrain, 180)
        );
    }
}
