package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube.EjectCubeAuto;
import frc.robot.commands.Autonomous.Positions.AutoArmToPickup;
import frc.robot.commands.Autonomous.Positions.AutoMoveArmToTop;
import frc.robot.commands.Autonomous.Positions.PickUpPieceAuto;
import frc.robot.commands.Drivetrain.TurnSetDegrees;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;

public class HighCubePickPieceAuto extends SequentialCommandGroup {
    Drivetrain drivetrain;
    EveryClaw claw;
    public HighCubePickPieceAuto(Drivetrain drivetrain, EveryClaw claw){
        super(
            new AutoMoveArmToTop(),
            new EjectCubeAuto(claw),
            new AutoBackArmRest(drivetrain, .65, 3),
            new TurnSetDegrees(drivetrain, 180),
            new AutoArmToPickup(),
            new PickUpPieceAuto(drivetrain, claw),
            new TurnSetDegrees(drivetrain, 180),
            new AutoBackArmRest(drivetrain, -.6, 3.5)

        );
    }
}