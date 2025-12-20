package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.commands.DriveCommands
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.SwerveDrive
import frc.team449.subsystems.drive.SwerveModuleIOSim
import frc.team449.subsystems.drive.gyro.GyroIO
import frc.team449.subsystems.drive.gyro.GyroIOSim
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger


class RobotContainer {

  // PDH
  val powerDistribution: PowerDistribution =
    PowerDistribution(
      RobotConstants.PDH_CAN_ID,
      PowerDistribution.ModuleType.kRev,
    )

  // driver/operator controllers
  val driveController: CommandXboxController = CommandXboxController(0)
  val opController: CommandXboxController = CommandXboxController(1)
  val characController: CommandXboxController = CommandXboxController(2)
  val testController: CommandXboxController = CommandXboxController(3)

  lateinit var driveSimulation: SwerveDriveSimulation
  lateinit var drive: SwerveDrive

  val autonomousCommand = PrintCommand("This is the autonomous command!")

  init {
    when (RobotConstants.CURRENT_MODE) {
      RobotConstants.Mode.SIM -> {
        driveSimulation = SwerveDriveSimulation(SwerveDrive.driveTrainSimulationConfig, Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

        drive = SwerveDrive(
          GyroIOSim(driveSimulation.gyroSimulation),
          SwerveModuleIOSim(driveSimulation.modules[0]),
          SwerveModuleIOSim(driveSimulation.modules[1]),
          SwerveModuleIOSim(driveSimulation.modules[2]),
          SwerveModuleIOSim(driveSimulation.modules[3]),
          driveSimulation::setSimulationWorldPose
        )
      }
      else -> {}
    }

    bindControls()
  }

  // controller bindings
  fun bindControls() {
    drive.defaultCommand =
      DriveCommands.joystickDrive(
        drive,
        { -driveController.leftY },
        { -driveController.leftX },
        { -driveController.rightX }
      )

    driveController
      .x()
      .onTrue(
        PrintCommand("X Button Pressed!")
      )
  }

  fun updateSimulation() {
    if (RobotConstants.CURRENT_MODE != RobotConstants.Mode.SIM) return

    SimulatedArena.getInstance().simulationPeriodic()
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.simulatedDriveTrainPose);
  }
}
