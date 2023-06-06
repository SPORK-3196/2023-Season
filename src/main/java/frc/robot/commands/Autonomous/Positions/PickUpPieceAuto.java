package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;
import frc.robot.commands.Autonomous.DriveBackwardsAuto;
import frc.robot.commands.Autonomous.Cube.IntakeCubeAuto;

public class PickUpPieceAuto extends ParallelCommandGroup{
    Drivetrain drivetrain;
    EveryClaw claw;
    public PickUpPieceAuto(Drivetrain drivetrain, EveryClaw claw) {
        super(
            new DriveBackwardsAuto(drivetrain, 2, -.3),
            new IntakeCubeAuto(claw, 2)
        );
    }
}