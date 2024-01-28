// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Manejo;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.Chasis;
import frc.robot.commands.Autonomo;

public class RobotContainer {
  private static final Chasis chasis = new Chasis();
  private static final XboxController driveControl = new XboxController(0);

  public RobotContainer() {

    chasis.setDefaultCommand(new Manejo(chasis, () -> driveControl.getLeftX(),
        () -> driveControl.getLeftY(), () -> driveControl.getRightX()));

    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake());
    NamedCommands.registerCommand("RunIntake", new RunIntake());

    configureBindings();
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Path1");

    return Commands.sequence(AutoBuilder.followPathWithEvents(path));
    
    // return new Autonomo(chasis);
  }
}
