package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.SuperstructureGoal
import kotlin.math.PI
import kotlin.math.abs

class AutoScorePathfinder(val robot: Robot, private val endPose: Pose2d) {
  private var ADStar = LocalADStar()

  private val velXPub: DoublePublisher
  private val velYPub: DoublePublisher
  private val velRotationPub: DoublePublisher
  private val setpointPub: BooleanPublisher
  private val rotPub: BooleanPublisher
  private val autodistancePub: BooleanPublisher
  private val admagpub: DoublePublisher
  private val distpub: DoublePublisher
  private lateinit var path: PathPlannerPath

  var velocityX = 0.0
  var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var inPIDDistance = false
  private var tolerance = 0.15
  private var pidDistance = 1.1
  var atSetpoint = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0
  private val pidOffsetTime = 0.5

  private var thetaController: PIDController = PIDController(5.0, 0.5, 0.0)
  private var xController = PIDController(7.0, 2.0, 0.1)
  private var yController = PIDController(7.0, 2.0, 0.1)
  var distance: Double = 100.0
  private var adMag = 1.0
  private var pidMag = 0.2
  private val rotTol = 0.085
  private val adDecRate = 0.04

  init {
    timer.restart()
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathMag/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathMag/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathMag/pathRotation").publish()
    setpointPub = NetworkTableInstance.getDefault().getBooleanTopic("/pathMag/atSetpoint").publish()
    rotPub = NetworkTableInstance.getDefault().getBooleanTopic("/pathMag/atRotSetpoint").publish()
    autodistancePub = NetworkTableInstance.getDefault().getBooleanTopic("/pathMag/inPIDDistance").publish()
    distpub = NetworkTableInstance.getDefault().getDoubleTopic("/pathMag/distance").publish()
    admagpub = NetworkTableInstance.getDefault().getDoubleTopic("/pathMag/adMag").publish()

    PathPlannerPath.clearCache()
    xController.reset()
    xController.reset()
    thetaController.reset()

    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(rotTol)
    xController.setTolerance(tolerance)
    yController.setTolerance(tolerance)
  }

  fun runSetup() {
    timer.restart()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    ADStar.setGoalPosition(endPose.translation)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
  }

  fun atRotSetpoint(): Boolean {
    return abs(robot.poseSubsystem.pose.rotation.radians - endPose.rotation.radians) < rotTol
  }

  fun pathfind() {
    val currentTime = timer.get()
    ADStar.setStartPosition(endPose.translation)
    distance = robot.poseSubsystem.pose.translation.getDistance(endPose.translation)
    if (distance < pidDistance) {
      if (!inPIDDistance) {
        inPIDDistance = true
        xController.setpoint = endPose.translation.x
        yController.setpoint = endPose.translation.y
      }
      adMag -= adDecRate
      if (adMag < 0) {
        adMag = 0.0
      }
      if (distance < tolerance) {
        atSetpoint = true
      }
    } else {
      inPIDDistance = false
      if (distance < 0.75) {
        pidMag += adDecRate
        if (pidMag > 1) {
          pidMag = 1.0
        }
      }
    }
    if (!atSetpoint) {
      val newPath: PathPlannerPath? = ADStar.getCurrentPath(
        PathConstraints(
          AutoScoreCommandConstants.MAX_LINEAR_SPEED,
          AutoScoreCommandConstants.MAX_ACCEL,
          5 * 2 * Math.PI,
          RobotConstants.ROT_RATE_LIMIT
        ),
        GoalEndState(0.0, endPose.rotation)
      )
      if (newPath == null) {
        pathNull = true
      } else {
        path = newPath
        pathNull = false
      }
      if (!pathNull) {
        val pathList = path.pathPoses.toTypedArray<Pose2d>()
        val newTraj: PathPlannerTrajectory? = path.generateTrajectory(
          robot.drive.currentSpeeds,
          Rotation2d(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)),
          RobotConfig.fromGUISettings()
        )
        if (newTraj == null) {
          trajectoryNull = true
        } else {
          trajectory = newTraj
          trajectoryNull = false
        }
        if (!trajectoryNull) {
          expectedTime = trajectory.totalTimeSeconds
          trajValid = (pathList[0] != endPose)
          startTime = currentTime
        }
      }
      if (!trajectoryNull && trajValid) {
        if (inPIDDistance || currentTime - startTime + pidOffsetTime > expectedTime) {
          xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.translation.x)
          yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.translation.y)
        } else {
          expectedTime = trajectory.totalTimeSeconds
          trajectory.sample(currentTime - startTime + pidOffsetTime).pose.let {
            xController.setpoint = (it.translation.x)
            yController.setpoint = (it.translation.y)
            xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.translation.x)
            yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.translation.y)
          }
        }
        trajectory.sample(currentTime - startTime).fieldSpeeds.let {
          val trajSpeeds = ChassisSpeeds(
            it.vxMetersPerSecond,
            it.vyMetersPerSecond,
            it.omegaRadiansPerSecond
          )
          velocityX = trajSpeeds.vxMetersPerSecond * adMag
          velocityY = trajSpeeds.vyMetersPerSecond * adMag
          rotation = 0.0
        }
      }
    }
    xPIDSpeed *= pidMag
    yPIDSpeed *= pidMag

    rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians), endPose.rotation.radians)
    if (atRotSetpoint()) {
      rotation = 0.0
    }
    if (atSetpoint) {
      robot.poseSubsystem.setPathMag(ChassisSpeeds(0.0, 0.0, rotation))
    } else {
      val fieldRelative = fromFieldRelativeSpeeds(
        ChassisSpeeds(
          MathUtil.clamp((velocityX + xPIDSpeed), -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED),
          MathUtil.clamp((velocityY + yPIDSpeed), -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED),
          MathUtil.clamp(rotation, -AutoScoreCommandConstants.MAX_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_SPEED)
        ),
        robot.poseSubsystem.pose.rotation
      )
      robot.poseSubsystem.setPathMag(fieldRelative)
    }
    setpointPub.set(atSetpoint)
    rotPub.set(atRotSetpoint())
    autodistancePub.set(inPIDDistance)
    velXPub.set(xPIDSpeed)
    velYPub.set(yPIDSpeed)
    velRotationPub.set(rotation)
    admagpub.set(adMag)
    distpub.set(distance)
  }
}

class EmptyDrive(drive: SwerveDrive) : Command() {
  init {
    addRequirements(drive)
  }
  override fun isFinished(): Boolean {
    return true
  }
}

// goal should be a premove state
class AutoScoreWrapperCommand(
  val robot: Robot,
  command: AutoScorePathfinder,
  private val goal: SuperstructureGoal.SuperstructureState
) :
  Command() {

  private val asPathfinder = command
  private var reefAlignCommand: Command = InstantCommand()
  private var usingReefAlign = false

  override fun initialize() {
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.schedule()
    asPathfinder.runSetup()
  }

  override fun execute() {
    if (asPathfinder.atSetpoint && asPathfinder.atRotSetpoint() && !usingReefAlign) {
      usingReefAlign = true
      reefAlignCommand = SimpleReefAlign(robot.drive, robot.poseSubsystem)
      reefAlignCommand.schedule()
      //premove to goal
      robot.superstructureManager.requestGoal(goal).schedule()
    } else {
      asPathfinder.pathfind()
    }
  }

  override fun isFinished(): Boolean {
    if (usingReefAlign) {
      return reefAlignCommand.isFinished
    }
    return false
  }
}
