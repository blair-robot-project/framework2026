package frc.team449

import au.grapplerobotics.CanBridge
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class Robot : LoggedRobot() {
  init {
    println("Started robotInit.")

    CanBridge.runTCP()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)
    DriverStation.silenceJoystickConnectionWarning(true)

    when (RobotConstants.CURRENT_MODE) {
      RobotConstants.Mode.REAL -> {
        Logger.addDataReceiver(WPILOGWriter())
        Logger.addDataReceiver(NT4Publisher())
      }

      RobotConstants.Mode.SIM -> {
        Logger.addDataReceiver(NT4Publisher())
      }

      RobotConstants.Mode.REPLAY -> {
        this.setUseTiming(false)
        val logPath: String = LogFileUtil.findReplayLog()
        Logger.setReplaySource(WPILOGReader(logPath))
        Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
      }
    }

    Logger.start()
  }

  private val robotContainer = RobotContainer()

  override fun driverStationConnected() {
  }

  override fun robotPeriodic() {
    // high priority (real-time) thread for loop timing
    Threads.setCurrentThreadPriority(true, 99)

    CommandScheduler.getInstance().run()

    // return thread to low priority (standard)
    Threads.setCurrentThreadPriority(false, 10)
  }

  override fun autonomousInit() {
    val autonomousCommand: Command = robotContainer.autonomousCommand

    autonomousCommand.schedule()
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {}

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {}

  override fun disabledPeriodic() {}

  override fun testInit() {}

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    robotContainer.updateSimulation()
  }
}
