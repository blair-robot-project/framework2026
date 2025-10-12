package frc.team449

import au.grapplerobotics.CanBridge
import com.ctre.phoenix6.SignalLogger
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.auto.Routines
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.vision.VisionConstants
import org.littletonrobotics.urcl.URCL
import javax.xml.crypto.Data
import kotlin.math.*

@Logged
/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot() {
  private val robot = Robot()

  val routines = Routines(robot)

  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot.characController, robot.testController, robot)

  init {
    CanBridge.runTCP()

    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    // Don't complain about joysticks if there aren't going to be any
    DriverStation.silenceJoystickConnectionWarning(true)

    // Generate Auto Routines
    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    // Adds Auto Routines to Auto Chooser
    routines.addOptions(robot.autoChooser)

    // Adds Auto Selection into Smart Dashboard
    SmartDashboard.putData("Auto Chooser", robot.autoChooser)

    // While in Autonomous Period, run the selected auto until autos are over, then cancel command.
    RobotModeTriggers.autonomous().whileTrue(robot.autoChooser.selectedCommandScheduler())
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    controllerBinder.bindButtons()

    SmartDashboard.putData("Field", robot.field)

    // CTRE Logger
    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()
    // REV Logger
    URCL.start()

    DataLogManager.start()
    Epilogue.bind(this)
  }

  override fun driverStationConnected() {
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    // Robot Drive Logging
    robot.field.robotPose = robot.poseSubsystem.pose
    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command. */
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    robot.superstructureManager.requestGoal(SuperstructureGoal.STOW).schedule()

    (robot.light.currentCommand ?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
    robot.superstructureManager.logData()
  }

  override fun disabledInit() {
    robot.drive.stop()
  }

  override fun disabledPeriodic() {}

  override fun testInit() {}

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    RobotVisual.update()

    // Superstructure Simulation
    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odometryPose)
    }

    VisionConstants.VISION_SIM.debugField
      .getObject("EstimatedRobot")
      .pose = robot.poseSubsystem.pose
  }
}
