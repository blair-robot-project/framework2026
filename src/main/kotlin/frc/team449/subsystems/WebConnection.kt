package frc.team449.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringSubscriber
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.subsystems.superstructure.SuperstructureGoal

class WebConnection(val robot: Robot) : SubsystemBase() {
  private val instance = NetworkTableInstance.getDefault()
  private val webComTable = instance.getTable("webcom")
  private val allianceTopic = webComTable.getStringTopic("Alliance")
  private val isDoneTopic = webComTable.getBooleanTopic("isDone")
  private val commandSubscriber: StringSubscriber = webComTable.getStringTopic("Command").subscribe("none")
  private val commandPublisher: StringPublisher = webComTable.getStringTopic("Command").publish()
  private val isDonePublish = isDoneTopic.publish()
  private val isDoneSub = isDoneTopic.subscribe(true)
  private val alliancePublish = allianceTopic.publish()
  private var ntCommandInput = "none"
  private var autoScore = AutoScoreCommands(robot)
  private var webAppCommand : Command = InstantCommand()
  private val pivotAngleIncrease = 0.035
  private val wristAngleIncrease = 0.04
  private val elevatorIncrease = 0.028592106


  fun setUpNT() {
    instance.startClient4("localhost")
    instance.setServerTeam(449)
    isDonePublish.set(false)
    commandPublisher.set("none")
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
      isDonePublish.set(false)

      webAppCommand = when (ntCommandInput) {
        "processor" -> autoScore.getProcessorCommand().andThen(InstantCommand({isDonePublish.set(false)}))
        "netRed" -> autoScore.getNetCommand(true).andThen(InstantCommand({isDonePublish.set(false)}))
        "netBlue" -> autoScore.getNetCommand(false).andThen(InstantCommand({isDonePublish.set(false)}))
        "cancel" -> autoScore.cancelCommand().andThen(WaitCommand(0.25)).andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
        "score" -> autoScore.scoreCommand().andThen(WaitCommand(0.25)).andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
        "pivotForward" -> robot.pivot.setPosition(robot.pivot.positionSupplier.get()-pivotAngleIncrease)
        "pivotBack" -> robot.pivot.setPosition(robot.pivot.positionSupplier.get()+pivotAngleIncrease)
        "wristForward" -> robot.wrist.setPosition(robot.wrist.positionSupplier.get()-wristAngleIncrease)
        "wristBack" -> robot.wrist.setPosition(robot.wrist.positionSupplier.get()+wristAngleIncrease)
        "elevatorUp" -> robot.elevator.setPosition(robot.elevator.positionSupplier.get()+elevatorIncrease)
        "elevatorDown" -> robot.elevator.setPosition(robot.elevator.positionSupplier.get()-elevatorIncrease)
        else -> {
          //format will be l_ location__
          isDonePublish.set(false)
          val level = ntCommandInput.slice(0..1)
          val location = ntCommandInput.slice(3..<ntCommandInput.length)
          val reefLocation = when (location) {
            "locationA" -> FieldConstants.REEF_LOCATIONS[0]
            "locationB" -> FieldConstants.REEF_LOCATIONS[1]
            "locationC" -> FieldConstants.REEF_LOCATIONS[2]
            "locationD" -> FieldConstants.REEF_LOCATIONS[3]
            "locationE" -> FieldConstants.REEF_LOCATIONS[4]
            "locationF" -> FieldConstants.REEF_LOCATIONS[5]
            "locationG" -> FieldConstants.REEF_LOCATIONS[6]
            "locationH" -> FieldConstants.REEF_LOCATIONS[7]
            "locationI" -> FieldConstants.REEF_LOCATIONS[8]
            "locationJ" -> FieldConstants.REEF_LOCATIONS[9]
            "locationK" -> FieldConstants.REEF_LOCATIONS[10]
            "locationL" -> FieldConstants.REEF_LOCATIONS[11]
            else -> FieldConstants.REEF_LOCATIONS[0]
          }
          val coralLevel = when (level) {
            "l1" -> SuperstructureGoal.L1
            "l2" -> SuperstructureGoal.L2
            "l3" -> SuperstructureGoal.L3
            "l4" -> SuperstructureGoal.L4
            else -> SuperstructureGoal.L1
          }
          autoScore.getReefCommand(reefLocation, coralLevel)
        }
      }
      webAppCommand.schedule()
      commandPublisher.set("none")
    } else {
      if(!isDoneSub.get()) {
        if(autoScore.currentCommandFinished()) {
          isDonePublish.set(true)
        }
      }

      if(autoScore.waitingForScore) {
        robot.superstructureManager.holdAll()
      }
    }
  }
}
