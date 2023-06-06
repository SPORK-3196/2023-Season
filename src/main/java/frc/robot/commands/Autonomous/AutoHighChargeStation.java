package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Cube.EjectCubeAuto;
import frc.robot.commands.Autonomous.Positions.AutoArmToRestPos;
import frc.robot.commands.Autonomous.Positions.AutoMoveArmToTop;
import frc.robot.commands.Autonomous.Positions.DockOnStation;
import frc.robot.commands.Autonomous.Positions.DriveBackOnChargeStation;
import frc.robot.commands.Autonomous.Positions.MoveBackBitTurn180;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;


public class AutoHighChargeStation extends SequentialCommandGroup{
    Drivetrain drivetrain;
    EveryClaw claw;

    public AutoHighChargeStation(Drivetrain drivetrain, EveryClaw claw){
        super(
            new AutoMoveArmToTop(),
            new EjectCubeAuto(claw),
            new AutoArmToRestPos(),
            new MoveBackBitTurn180(drivetrain),
            new DriveBackOnChargeStation(drivetrain, -.62),
            new DockOnStation(drivetrain),
            new DriveBackwardsAuto(drivetrain, .3, .5)
        );
    }
}