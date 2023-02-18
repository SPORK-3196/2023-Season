package frc.robot.commands.Autonomous.Positions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.Drivetrain.DistanceDrive;
import frc.robot.commands.Drivetrain.Turn90RightDegrees;

public class Square extends SequentialCommandGroup {
    Drivetrain drivetrain;
    public Square(Drivetrain drivetrain){
        super(
            new DistanceDrive(drivetrain, 5),
            new Turn90RightDegrees(drivetrain, 5),
            new DistanceDrive(drivetrain, 5),
            new Turn90RightDegrees(drivetrain, 5),
            new DistanceDrive(drivetrain, 5),
            new Turn90RightDegrees(drivetrain, 5),
            new DistanceDrive(drivetrain, 5),
            new Turn90RightDegrees(drivetrain, 5)
        );
    }
}
