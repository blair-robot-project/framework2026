package frc.team449
//
// import com.ctre.phoenix6.SignalLogger
// import edu.wpi.first.math.geometry.Pose2d
// import edu.wpi.first.math.geometry.Rotation2d
// import edu.wpi.first.units.Units.*
// import edu.wpi.first.units.measure.Voltage
// import edu.wpi.first.wpilibj.DriverStation
// import edu.wpi.first.wpilibj2.command.*
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
// import frc.team449.RobotConstants
// import frc.team449.subsystems.swerve.SwerveSim
// import frc.team449.subsystems.swerve.WheelRadiusCharacterization
// import kotlin.jvm.optionals.getOrNull
// import kotlin.math.PI
// import kotlin.random.Random
//
class ControllerBindings()
//    private val driveController: CommandXboxController,
//    private val mechanismController: CommandXboxController,
//    private val characterizationController: CommandXboxController,
//    private val testController: CommandXboxController,
//    private val robot: RobotContainer
// ) {
//
//  private fun robotBindings() {
//  }
//
//  private fun characterizationBindings() {
//  }
//
//  private fun nonRobotBindings() {
//    // slowDrive()
//  }
//
// /** driver controller dpad **/
//  // povU
//
//  private fun slowDrive() {
//    driveController
//      .rightBumper()
//      .onTrue(
//        InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
//          .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 })),
//      ).onFalse(
//        InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
//          .andThen(
//            InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED }),
//          ),
//      )
//  }
//
//  /** Characterization functions */
//  private fun wheelRadiusCharacterization() {
//    characterizationController.leftTrigger().onTrue(
//      WheelRadiusCharacterization(robot.drive, robot.poseSubsystem),
//    )
//  }
//
//  private fun driveCharacterization() {
//    val driveRoutine =
//      SysIdRoutine(
//        SysIdRoutine.Config(
//          Volts.of(1.0).per(Second),
//          Volts.of(2.0),
//          Seconds.of(4.0),
//        ) { state -> SignalLogger.writeString("state", state.toString()) },
//        Mechanism(
//          { voltage: Voltage -> robot.drive.setVoltage(-voltage.`in`(Volts)) },
//          null,
//          robot.drive,
//        ),
//      )
//
//    // Quasistatic Forwards
//    characterizationController.povUp().onTrue(
//      driveRoutine.quasistatic(SysIdRoutine.Direction.kForward),
//    )
//
//    // Quasistatic Reverse
//    characterizationController.povDown().onTrue(
//      driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
//    )
//
//    // Dynamic Forwards
//    characterizationController.povRight().onTrue(
//      driveRoutine.dynamic(SysIdRoutine.Direction.kForward),
//    )
//
//    // Dynamic Reverse
//    characterizationController.povLeft().onTrue(
//      driveRoutine.dynamic(SysIdRoutine.Direction.kReverse),
//    )
//  }
//
//  /** Try not to touch, just add things to the robot or nonrobot bindings */
//  fun bindButtons() {
//    println("Binding Buttons")
//    nonRobotBindings()
//    println("\tBound non robot bindings")
//    robotBindings()
//    println("\tBound robot bindings")
//    characterizationBindings()
//    println("\tBound characterization bindings")
//  }
// }
