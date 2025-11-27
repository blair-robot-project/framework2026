package frc.team449

import choreo.auto.AutoChooser
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.hardwaremanagers.RobotConstants
import frc.team449.hardwaremanagers.drive.swerve.SwerveDrive
import frc.team449.commands.SwerveOrthogonalCommand
import frc.team449.hardware.light.Light.Companion.createLight
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.hardwaremanagers.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.hardware.AHRS
// import frc.team449.hardwaremanagers.superstructure.BIT.BuiltInTests

@Logged
class Robot {

  // Driver/Operator Controllers
  @get:NotLogged
  val driveController: CommandXboxController = CommandXboxController(0)

  @get:NotLogged
  val mechController: CommandXboxController = CommandXboxController(1)

  @get:NotLogged
  val characController: CommandXboxController = CommandXboxController(2)

  @get:NotLogged
  val testController: CommandXboxController = CommandXboxController(3)

  val field = Field2d()

  // NavX
  val ahrs: AHRS = AHRS()

  // Instantiate/declare PDP and other stuff here
  val powerDistribution: PowerDistribution =
    PowerDistribution(
      RobotConstants.PDH_CAN,
      PowerDistribution.ModuleType.kRev,
    )

  @get:NotLogged
  val drive: SwerveDrive = SwerveDrive.createSwerveKraken(field)

  val autoChooser = AutoChooser()

  @get:NotLogged
  val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field)

  @get:NotLogged
  val driveCommand: SwerveOrthogonalCommand = SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

  @get:NotLogged
  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  val light = createLight()
}
