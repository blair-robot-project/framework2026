package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.commands.SwerveRequestCommand
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

class RobotContainer {

  // PDH
  val powerDistribution: PowerDistribution =
    PowerDistribution(
      Constants.PDH_CAN_ID,
      PowerDistribution.ModuleType.kRev,
    )

  // driver/op controllers
  val driveController: CommandXboxController = CommandXboxController(0)
  val opController: CommandXboxController = CommandXboxController(1)
  val characController: CommandXboxController = CommandXboxController(2)
  val testController: CommandXboxController = CommandXboxController(3)

  lateinit var driveSimulation: SwerveDriveSimulation
  lateinit var drive: DriveSubsystem

  val autonomousCommand = PrintCommand("This is the autonomous command!")

  init {
    when (Constants.CURRENT_MODE) {
      Constants.Mode.SIM -> {
        drive = DriveSubsystem(
          DriveIOSim(
            arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight),
            TunerConstants.DrivetrainConstants,
          )
        )
      }
      else -> {}
    }

    bindControls()
  }

  // controller bindings
  fun bindControls() {
    drive.defaultCommand =
      SwerveRequestCommand(
        drive,
        driveController::getLeftY,
        driveController::getLeftX,
        driveController::getRightX
      )

    driveController
      .x()
      .onTrue(
        PrintCommand("X Button Pressed!")
      )
  }
}
