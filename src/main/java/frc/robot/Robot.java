// © 2025 Excalibur FRC. All rights reserved.
// This file is part of ExcaLIb and may not be copied, modified,
// or distributed without permission, except as permitted by license.
// learn more at - https://github.com/ExaliburFRC/ExcaLIb

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.excalib.additional_utilities.periodics.PeriodicScheduler;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;


    public Robot() {
        m_robotContainer = new RobotContainer();
        Monologue.setupMonologue(this, "Robot", false, true);
        PeriodicScheduler scheduler = PeriodicScheduler.getInstance();
    }

    @Override
    public void robotPeriodic() {
        Monologue.updateAll();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null)
            m_autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
