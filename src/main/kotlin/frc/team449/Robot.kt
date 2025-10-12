package frc.team449

import choreo.auto.AutoChooser
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.superstructure.SuperstructureManager
import frc.team449.subsystems.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.system.AHRS
// import frc.team449.subsystems.superstructure.BIT.BuiltInTests

@Logged
class Robot {

  // Driver/Operator Controllers
  val driveController: CommandXboxController = CommandXboxController(0)
  val mechController: CommandXboxController = CommandXboxController(1)
  val characController: CommandXboxController = CommandXboxController(2)
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

  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  val light = createLight()
}
