package frc.team449

import choreo.auto.AutoChooser
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.commands.drive.SwerveDriveCommand
import frc.team449.config.RobotConstants
import frc.team449.config.SwerveConstants
import frc.team449.hardware.AHRS
import frc.team449.hardware.light.Light.Companion.createLight
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.hardwaremanagers.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.hardwaremanagers.drive.swerve.SwerveDrive
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.input.HolonomicOI
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig

@Logged
object Robot {

  // Driver/Operator Controllers
  @get:NotLogged
  val driveController: CommandXboxController = CommandXboxController(0)

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

  val drive: SwerveDrive = if (RobotBase.isReal()) SwerveDrive.createSwerveKraken() else SwerveDrive.createSwerveSim()

  val autoChooser = AutoChooser()

  @get:NotLogged
  val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field)

  val holonomicOi: HolonomicOI = HolonomicOI(RobotConstants.ROT_RATE_LIMIT, RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ROT_SPEED, RobotConstants.MAX_ACCEL)

  @get:NotLogged
  val driveCommand: SwerveDriveCommand = SwerveDriveCommand(drive, poseSubsystem, driveController.hid, holonomicOi, RobotConstants.FIELD_RELATIVE_ENABLED)

  @get:NotLogged
  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  val light = createLight()
}
