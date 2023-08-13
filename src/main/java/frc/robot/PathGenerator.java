package frc.robot;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.HighRungPlace;
import frc.robot.commands.Autonomous.MidRungAuto;
import frc.robot.commands.Autonomous.Cube.DropIntakeCubeAuto;
import frc.robot.commands.Autonomous.Cube.EjectCubeAuto;
import frc.robot.commands.Autonomous.Cube.IntakeCubeAuto;
import frc.robot.commands.Autonomous.Positions.AutoArmToPickup;
import frc.robot.commands.Autonomous.Positions.AutoArmToRestPos;
import frc.robot.commands.Autonomous.Positions.AutoMoveArmToTop;
import frc.robot.commands.Drivetrain.TurnSetDegrees;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.EveryClaw;

public class PathGenerator {
    public static RamseteAutoBuilder generateRamseteCommand(Drivetrain drivetrain, EveryClaw claw){
        drivetrain.frontLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.frontRight.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearLeft.setNeutralMode(NeutralMode.Brake);
        drivetrain.rearRight.setNeutralMode(NeutralMode.Brake);

        HashMap<String, Command> map = new HashMap<>();

        map.put("armToHigh", new AutoMoveArmToTop());
        map.put("dropCube", new EjectCubeAuto(claw));
        map.put("armToRest", new AutoArmToRestPos());
        map.put("intakeDown", new AutoArmToPickup());
        map.put("intakeCube", new IntakeCubeAuto(claw, 1));
        map.put("placePieceHigh", new HighRungPlace(drivetrain, claw));
        map.put("placePieceMid", new HighRungPlace(drivetrain, claw));
        map.put("pickupPiece", new DropIntakeCubeAuto(claw));
        map.put("placePieceMid", new MidRungAuto(drivetrain, claw));
        map.put("turn180degrees", new TurnSetDegrees(drivetrain, 180));

        return new RamseteAutoBuilder(drivetrain::getPose,
        drivetrain::resetPose,
        new RamseteController(2, .7),
        Constants.DrivetrainConstants.m_2022DifferentialDriveKinematics,
        drivetrain.m_feedforward,
        drivetrain::motorWheelSpeeds,
        new PIDConstants(Constants.DrivetrainConstants.m_2022kP, 0, 0),
        drivetrain::tankDriveVolts,
        map,
        true,
        drivetrain
        );
    }
} 