// © 2025 Excalibur FRC. All rights reserved.
// This file is part of ExcaLIb and may not be copied, modified,
// or distributed without permission, except as permitted by license.
// learn more at - https://github.com/ExaliburFRC/ExcaLIb

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
