package frc.team449

import au.grapplerobotics.CanBridge
import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.epilogue.Epilogue
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.auto.Routines
import frc.team449.config.VisionConstants
import frc.team449.hardwaremanagers.drive.swerve.SwerveSim
import frc.team449.sim.Scoreboard
import frc.team449.util.Clock
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.urcl.URCL
import kotlin.math.*

/** The main class of the robot, constructs all the hardwaremanagers
 * and initializes default commands . */
@Logged
class RobotLoop : TimedRobot() {
  private val robot = Robot
  private val clock = Clock

  override fun robotInit() {
    CanBridge.runTCP()

    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    // Don't complain about joysticks if there aren't going to be any
    DriverStation.silenceJoystickConnectionWarning(true)

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    // todo, should this be happening here?
    SmartDashboard.putData("Field", robot.poseSubsystem.field)

    // CTRE Logger
    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()
    // REV Logger
    URCL.start()

    DataLogManager.start()
    Epilogue.bind(this)

    robot.robotInit()
  }

  override fun driverStationConnected() {
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    // Robot Drive Logging
    robot.field.robotPose = robot.poseSubsystem.pose
    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose
  }

  override fun autonomousInit() {}

  override fun autonomousPeriodic() {}

  override fun teleopInit() {}

  override fun teleopPeriodic() {}

  override fun disabledInit() {}

  override fun disabledPeriodic() {}

  override fun testInit() {}

  override fun testPeriodic() {}

  var lunites: StructArrayPublisher<Pose3d?> = NetworkTableInstance.getDefault()
    .getStructArrayTopic<Pose3d?>("Lunites", Pose3d.struct).publish()

  override fun simulationInit() {
  }

  override fun simulationPeriodic() {
    // MapleSim
    SimulatedArena.getInstance().simulationPeriodic()
    Scoreboard.display()

    val lunitePoses: Array<Pose3d> = SimulatedArena.getInstance().getGamePiecesArrayByType("Lunite")
    lunites.set(lunitePoses)

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
