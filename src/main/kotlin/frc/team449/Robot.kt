package frc.team449

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.epilogue.NotLogged
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.commands.drive.SwerveDriveCommand
import frc.team449.commands.drive.WheelRadiusCharacterization
import frc.team449.config.RobotConstants
import frc.team449.hardwaremanagers.PoseSubsystem
import frc.team449.hardwaremanagers.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.hardwaremanagers.drive.swerve.SwerveDrive
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager
import frc.team449.hardwaremanagers.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.input.HolonomicOI
import frc.team449.sim.Lunite

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

  val autoChooser = AutoChooser()

  @get:NotLogged
  val poseSubsystem: PoseSubsystem = createPoseSubsystem(drive)

  val holonomicOi: HolonomicOI = HolonomicOI(RobotConstants.ROT_RATE_LIMIT, RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ROT_SPEED, RobotConstants.MAX_ACCEL)

  @get:NotLogged
  val driveCommand: SwerveDriveCommand = SwerveDriveCommand(drive, poseSubsystem, driveController.hid, holonomicOi, RobotConstants.FIELD_RELATIVE_ENABLED)

  val autoFactory = AutoFactory(
    poseSubsystem::pose,
    poseSubsystem::resetOdometry,
    drive::followTrajectory,
    true,
    drive
  )
  val commands = Commands(this)
  val routines = Routines(autoFactory)

  @get:NotLogged
  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  fun bindAutoRoutines() {
    // add routines
    autoChooser.addRoutine("Do nothing", routines::doNothing)

    // Adds Auto Selection into Smart Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser)
    // While in Autonomous Period, run the selected auto until autos are over, then cancel command.
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler())
  }

  private fun bindDriveController(controller: CommandXboxController) {
    controller.rightBumper().onTrue(commands.slowDrive()).onFalse(commands.restoreDriveSpeed())

    controller.a().onTrue(commands.pointToRight())

    controller.povUp().onTrue(commands.resetGyro())

    if (RobotBase.isSimulation()) {
      controller.a().onTrue(commands.resetOdometrySim())
      controller.x().onTrue(
        runOnce({
          Lunite.launchLunite(robot, Translation3d(), Degrees.of(45.0), MetersPerSecond.of(5.0))
        })
      )
    }
  }

  private fun bindCharacterizationController(controller: CommandXboxController, commands: Commands) {
    controller.leftTrigger().onTrue(
      commands.wheelRadiusCharacterization()
    )

    controller.povUp().onTrue(
      commands.driveCharacterization().quasistatic(SysIdRoutine.Direction.kForward),
    )
    controller.povDown().onTrue(
      commands.driveCharacterization().quasistatic(SysIdRoutine.Direction.kReverse),
    )
    controller.povRight().onTrue(
      commands.driveCharacterization().dynamic(SysIdRoutine.Direction.kForward),
    )
    controller.povLeft().onTrue(
      commands.driveCharacterization().dynamic(SysIdRoutine.Direction.kReverse),
    )
  }

  fun robotInit() {
    drive.defaultCommand = driveCommand

    bindAutoRoutines()

    bindDriveController(driveController)
  }
}
