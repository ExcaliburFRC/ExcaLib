// © 2025 Excalibur FRC. All rights reserved.
// This file is part of ExcaLIb and may not be copied, modified,
// or distributed without permission, except as permitted by license.
// learn more at - https://github.com/ExaliburFRC/ExcaLIbb

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
