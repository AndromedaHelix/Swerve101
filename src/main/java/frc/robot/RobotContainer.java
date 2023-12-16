// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Manejo;
import frc.robot.subsystems.Chasis;
import frc.robot.commands.Autonomo;

public class RobotContainer {
  private static final Chasis chasis = new Chasis();
  private static final XboxController driveControl = new XboxController(0);

  public RobotContainer() {

    chasis.setDefaultCommand(new Manejo(chasis, () -> driveControl.getLeftX(), 
                                  () -> driveControl.getLeftY(), () -> driveControl.getRightX()));
  
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new Autonomo(chasis);
  }
}
