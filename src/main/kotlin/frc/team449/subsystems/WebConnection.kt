package frc.team449.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringSubscriber
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.subsystems.superstructure.SuperstructureGoal

class WebConnection(val robot: Robot) : SubsystemBase() {
  private val instance = NetworkTableInstance.getDefault()
  private val webComTable = instance.getTable("webcom")
  private val allianceTopic = webComTable.getStringTopic("Alliance")
  private val commandSubscriber: StringSubscriber = webComTable.getStringTopic("Command").subscribe("none")
  private val commandPublisher: StringPublisher = webComTable.getStringTopic("Command").publish()
  private val alliancePublish = allianceTopic.publish()
  private val movingPublish = webComTable.getBooleanTopic("Moving").publish()
  private var ntCommandInput = "none"
  private var autoScore = AutoScoreCommands(robot)
  private var webAppCommand : Command = InstantCommand()
  private val elevatorIncrease = 0.028592106


  fun setUpNT() {
    instance.startClient4("localhost")
    instance.setServerTeam(449)
    commandPublisher.set("none")
    movingPublish.set(false)
    val alliance = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) "Red" else "Blue"
    alliancePublish.set(alliance)
  }

  fun closeServer() {
    instance.stopServer()
    instance.close()
  }

  override fun periodic() {
    ntCommandInput = commandSubscriber.get().toString()

    if (ntCommandInput != "none") {
      println("command received: $ntCommandInput")
      webAppCommand = when (ntCommandInput) {
        "processor" -> autoScore.getProcessorCommand()
        "netRed" -> autoScore.getNetCommand(true)
        "netBlue" -> autoScore.getNetCommand(false)
        "cancel" -> autoScore.cancelCommand().andThen(WaitCommand(0.25)).andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
        "elevatorUp" -> robot.elevator.setPosition(robot.elevator.positionSupplier.get()+elevatorIncrease)
        "elevatorDown" -> robot.elevator.setPosition(robot.elevator.positionSupplier.get()-elevatorIncrease)
        else -> {
          //format will be l_ pair_
          val level = ntCommandInput.slice(0..1)
          val pair = ntCommandInput.slice(3..<ntCommandInput.length)
          val reefLocations = when (pair) {
            "pair1" -> listOf(
              FieldConstants.REEF_LOCATIONS[0],
              FieldConstants.REEF_LOCATIONS[1]
            )
            "pair2" -> listOf(
              FieldConstants.REEF_LOCATIONS[2],
              FieldConstants.REEF_LOCATIONS[3]
            )
            "pair3" -> listOf(
              FieldConstants.REEF_LOCATIONS[4],
              FieldConstants.REEF_LOCATIONS[5]
            )
            "pair4" -> listOf(
              FieldConstants.REEF_LOCATIONS[6],
              FieldConstants.REEF_LOCATIONS[7]
            )
            "pair5" -> listOf(
              FieldConstants.REEF_LOCATIONS[8],
              FieldConstants.REEF_LOCATIONS[9]
            )
            "pair6" -> listOf(
              FieldConstants.REEF_LOCATIONS[10],
              FieldConstants.REEF_LOCATIONS[11]
            )
            else -> listOf(
              FieldConstants.REEF_LOCATIONS[0],
              FieldConstants.REEF_LOCATIONS[1]
            )
          }
          val coralLevel = when (level) {
            "l1" -> SuperstructureGoal.L1
            "l2" -> SuperstructureGoal.L2
            "l3" -> SuperstructureGoal.L3
            "l4" -> SuperstructureGoal.L4
            else -> SuperstructureGoal.L1
          }
          when (level) {
            "l1" -> robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE).schedule()
            "l2" -> robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE).schedule()
            "l3" -> robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE).schedule()
            "l4" -> robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).schedule()
          }
          autoScore.getReefCommand(reefLocations, coralLevel)
        }
      }
      webAppCommand.schedule()
      commandPublisher.set("none")
    }
    movingPublish.set(autoScore.moving)
  }
}
