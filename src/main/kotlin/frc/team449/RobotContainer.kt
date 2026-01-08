package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

    val autonomousCommand = PrintCommand("This is the autonomous command!")

    val drive: DriveSubsystem = when (Constants.CURRENT_MODE) {
        Mode.REAL -> DriveSubsystem(
            DriveIOHardware(
                TunerConstants.DrivetrainConstants,
                arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight),
            )
        )
        Mode.SIM -> DriveSubsystem(
            DriveIOSim(
                TunerConstants.DrivetrainConstants,
                arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight),
            )
        )
        Mode.REPLAY -> DriveSubsystem(
            object : DriveIO {}
        )
    }

    val bindings: ControllerBindings = ControllerBindings(this).apply {
        setDefaultCommands()
        bindControls()
    }
}
