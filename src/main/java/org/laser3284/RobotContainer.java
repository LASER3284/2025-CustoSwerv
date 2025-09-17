// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.laser3284;

import org.laser3284.Constants.OperatorConstants;
import org.laser3284.subsystems.swerve.Drive;
import org.laser3284.subsystems.swerve.Drive.Perspective;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Drive driveTrain = new Drive();

    private final CommandXboxController driverController =
        new CommandXboxController(OperatorConstants.DRIVE_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Create and register our bindings and commands for certain operations. Our
     * drive train, for example, needs four DoubleSuppliers for it's joystick
     * axes.
     */
    private void configureBindings() {
        driveTrain.setJoystickSuppliers(
                () -> { return driverController.getLeftX(); },
                () -> { return driverController.getLeftY(); },
                () -> { return driverController.getRightX(); },
                () -> { return driverController.getRightY(); }
            );
        // if the driver want's a button for changing perspective, that's also
        // possible
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
