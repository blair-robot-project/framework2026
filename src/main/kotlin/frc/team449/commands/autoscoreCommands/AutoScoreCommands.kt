package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal

class AutoScoreCommands(private val robot: Robot) {
  private var currentCommand: Command = InstantCommand()
  private var scoreCommand: Command = InstantCommand()
  var waitingForScore = false
  private var moving = false

  fun scoreCommand(): Command {
    waitingForScore = false
    return scoreCommand
  }

  fun currentCommandFinished(): Boolean {
    if(!currentCommand.isScheduled && moving) {
      moving = false
      return true
    }
    return false
  }

  fun getReefCommand(rl: AutoScoreCommandConstants.ReefLocation, cl: AutoScoreCommandConstants.CoralLevel): Command {
    return runOnce({
      moving = true
      val reefLocationPose =
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          when (rl) {
            AutoScoreCommandConstants.ReefLocation.Location1 -> AutoScoreCommandConstants.reef1PoseRed
            AutoScoreCommandConstants.ReefLocation.Location2 -> AutoScoreCommandConstants.reef2PoseRed
            AutoScoreCommandConstants.ReefLocation.Location3 -> AutoScoreCommandConstants.reef3PoseRed
            AutoScoreCommandConstants.ReefLocation.Location4 -> AutoScoreCommandConstants.reef4PoseRed
            AutoScoreCommandConstants.ReefLocation.Location5 -> AutoScoreCommandConstants.reef5PoseRed
            AutoScoreCommandConstants.ReefLocation.Location6 -> AutoScoreCommandConstants.reef6PoseRed
            AutoScoreCommandConstants.ReefLocation.Location7 -> AutoScoreCommandConstants.reef7PoseRed
            AutoScoreCommandConstants.ReefLocation.Location8 -> AutoScoreCommandConstants.reef8PoseRed
            AutoScoreCommandConstants.ReefLocation.Location9 -> AutoScoreCommandConstants.reef9PoseRed
            AutoScoreCommandConstants.ReefLocation.Location10 -> AutoScoreCommandConstants.reef10PoseRed
            AutoScoreCommandConstants.ReefLocation.Location11 -> AutoScoreCommandConstants.reef11PoseRed
            AutoScoreCommandConstants.ReefLocation.Location12 -> AutoScoreCommandConstants.reef12PoseRed
          }
        } else {
          when (rl) {
            AutoScoreCommandConstants.ReefLocation.Location1 -> AutoScoreCommandConstants.reef1PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location2 -> AutoScoreCommandConstants.reef2PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location3 -> AutoScoreCommandConstants.reef3PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location4 -> AutoScoreCommandConstants.reef4PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location5 -> AutoScoreCommandConstants.reef5PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location6 -> AutoScoreCommandConstants.reef6PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location7 -> AutoScoreCommandConstants.reef7PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location8 -> AutoScoreCommandConstants.reef8PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location9 -> AutoScoreCommandConstants.reef9PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location10 -> AutoScoreCommandConstants.reef10PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location11 -> AutoScoreCommandConstants.reef11PoseBlue
            AutoScoreCommandConstants.ReefLocation.Location12 -> AutoScoreCommandConstants.reef12PoseBlue
          }
        }
      val premoveGoal = when (cl) {
        AutoScoreCommandConstants.CoralLevel.L1 -> SuperstructureGoal.L1_PREMOVE
        AutoScoreCommandConstants.CoralLevel.L2 -> SuperstructureGoal.L2_PREMOVE
        AutoScoreCommandConstants.CoralLevel.L3 -> SuperstructureGoal.L3_PREMOVE
        AutoScoreCommandConstants.CoralLevel.L4 -> SuperstructureGoal.L4_PREMOVE
      }
      val scoreGoal = when (cl) {
        AutoScoreCommandConstants.CoralLevel.L1 -> SuperstructureGoal.L1
        AutoScoreCommandConstants.CoralLevel.L2 -> SuperstructureGoal.L2
        AutoScoreCommandConstants.CoralLevel.L3 -> SuperstructureGoal.L3
        AutoScoreCommandConstants.CoralLevel.L4 -> SuperstructureGoal.L4
      }
      robot.poseSubsystem.autoscoreCommandPose = reefLocationPose

      scoreCommand = if (!RobotBase.isSimulation()) {
        robot.superstructureManager.requestGoal(scoreGoal)
      } else {
        InstantCommand()
      }

      val premoveCommand = robot.superstructureManager.requestGoal(premoveGoal)

      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, reefLocationPose),
          premoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            waitingForScore = true
          })
        )
      currentCommand.schedule()
    })
  }

  fun getProcessorCommand(): Command {
    return runOnce({
      moving = true
      val processorPose = if (DriverStation.getAlliance().get() == Alliance.Red) AutoScoreCommandConstants.processorPoseRed else AutoScoreCommandConstants.processorPoseBlue
      scoreCommand = InstantCommand()
      val premoveCommand = InstantCommand()
      robot.poseSubsystem.autoscoreCommandPose = processorPose
      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, processorPose),
          premoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            waitingForScore = true
          })
        )
      currentCommand.schedule()
    })
  }

  fun getNetCommand(atRedSide: Boolean): Command {
    return runOnce({
      moving = true
      val netPose = Pose2d(
        AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.centerOfField * if (atRedSide) 1 else -1,
        robot.poseSubsystem.pose.translation.y,
        if (atRedSide) AutoScoreCommandConstants.netRotation2dRed else AutoScoreCommandConstants.netRotation2dBlue
      )
      scoreCommand = InstantCommand()
      val premoveCommand = InstantCommand()
      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, netPose),
          premoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            waitingForScore = true
          })
        )
      currentCommand.schedule()
    })
  }

  fun cancelCommand(): Command {
    return InstantCommand({
      moving = false
      currentCommand.cancel()
      //robot.drive.set(robot.drive.currentSpeeds)
      robot.drive.defaultCommand = robot.driveCommand
      waitingForScore = false
    })
  }
}
