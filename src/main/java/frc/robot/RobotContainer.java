// © 2025 Excalibur FRC. All rights reserved.
// This file is part of ExcaLIb and may not be copied, modified,
// or distributed without permission, except as permitted by license.
// learn more at - https://github.com/ExaliburFRC/ExcaLIb

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.AdjustableShooter;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    Cannon cannon = new Cannon();
    AdjustableShooter adjShooter = new AdjustableShooter();
    Swerve swerve = new Swerve();

    InterpolatingDoubleTreeMap velInterpolate = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap angleInterpolate = new InterpolatingDoubleTreeMap();

    CommandPS5Controller controller = new CommandPS5Controller(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        // move and shoot to target when cross is pressed
        controller.cross().onTrue(cannon.shootToPositionCommand(swerve::getRotationToTarget, swerve::getPitchToTarget));

        // toggle manual control when pressing square
        controller.square().toggleOnTrue(cannon.manualControlCommand(
                controller::getRightX, controller::getLeftY, controller.L1() //move the turret with joysticks, shoot with L1
        ));

        controller.triangle().onTrue(adjShooter.setShooterStateCommand(
                () -> velInterpolate.get(swerve.getDistanceFromTarget()),
                () -> angleInterpolate.get(swerve.getDistanceFromTarget())
        ));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
