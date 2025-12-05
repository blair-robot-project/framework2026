package frc.team449

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.commands.drive.SwerveDriveCommand
import frc.team449.config.RobotConstants
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.hardwaremanagers.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.hardwaremanagers.drive.swerve.SwerveDrive
import frc.team449.input.HolonomicOI

@Logged
object Robot {

  // Driver/Operator Controllers
  @get:NotLogged
  val driveController: CommandXboxController = CommandXboxController(0)

  // Instantiate/declare PDP and other stuff here
  val powerDistribution: PowerDistribution =
    PowerDistribution(
      RobotConstants.PDH_CAN,
      PowerDistribution.ModuleType.kRev,
    )

  @get:NotLogged
  val drive: SwerveDrive = if (RobotBase.isReal()) SwerveDrive.createSwerveKraken() else SwerveDrive.createSwerveSim()

  @get:NotLogged
  val poseSubsystem: PoseSubsystem = createPoseSubsystem(drive)

  val holonomicOi: HolonomicOI = HolonomicOI(
    RobotConstants.ROT_RATE_LIMIT,
    RobotConstants.MAX_LINEAR_SPEED,
    RobotConstants.MAX_ROT_SPEED,
    RobotConstants.MAX_ACCEL
  )

  @get:NotLogged
  val driveCommand: SwerveDriveCommand = SwerveDriveCommand(drive, poseSubsystem, driveController.hid, holonomicOi, RobotConstants.FIELD_RELATIVE_ENABLED)

  fun bindAutoRoutines() {
    // add routines
    Routines.addRoutine("Do nothing", Routines.doNothing())
  }

  private fun bindDriveController(controller: CommandXboxController) {
    controller.rightBumper().onTrue(Commands.slowDrive()).onFalse(Commands.restoreDriveSpeed())

    controller.a().onTrue(Commands.pointToRight())

    controller.povUp().onTrue(Commands.resetGyro())

    if (RobotBase.isSimulation()) {
      controller.a().onTrue(Commands.resetOdometrySim())
//      controller.x().onTrue(
//        runOnce({
//          Lunite.launchLunite(robot, Translation3d(), Degrees.of(45.0), MetersPerSecond.of(5.0))
//        })
//      )
    }
  }

  private fun bindCharacterizationController(controller: CommandXboxController) {
    controller.leftTrigger().onTrue(
      Commands.wheelRadiusCharacterization()
    )

    controller.povUp().onTrue(
      Commands.driveCharacterization().quasistatic(SysIdRoutine.Direction.kForward),
    )
    controller.povDown().onTrue(
      Commands.driveCharacterization().quasistatic(SysIdRoutine.Direction.kReverse),
    )
    controller.povRight().onTrue(
      Commands.driveCharacterization().dynamic(SysIdRoutine.Direction.kForward),
    )
    controller.povLeft().onTrue(
      Commands.driveCharacterization().dynamic(SysIdRoutine.Direction.kReverse),
    )
  }

  fun robotInit() {
    drive.defaultCommand = driveCommand

    bindAutoRoutines()

    bindDriveController(driveController)
  }
}
