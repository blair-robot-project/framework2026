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

  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var inPIDDistance = false
  private var pidDistance = 1.0
  private var tolerance = 0.15
  private val premoveDistance = 0.3

  var atSetpoint = false
  var atPremoveDistance = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0
  private val pidOffsetTime = 0.15

  private var thetaController: PIDController = PIDController(6.5, 0.8592, 0.0)
  private var xController = PIDController(7.0, 2.0, 0.0)
  private var yController = PIDController(7.0, 2.0, 0.0)
  var distance: Double
  private var adMag = 1.0
  private var pidMag = 0.0
  private val adDecRate = 0.07
  private val pidIncRate = 0.04

  init {
    timer.restart()
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/pathRotation").publish()
    setpointPub = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/atSetpoint").publish()
    rotPub = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/atRotSetpoint").publish()
    autodistancePub = NetworkTableInstance.getDefault().getBooleanTopic("/pathfinder/inPIDDistance").publish()
    distpub = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/distance").publish()

    xController.reset()
    yController.reset()
    xController.setTolerance(0.0)
    yController.setTolerance(0.0)

    thetaController.reset()
    thetaController.setpoint = endPose.rotation.radians
    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(0.0)

    adMag = 1.0
    admagpub = NetworkTableInstance.getDefault().getDoubleTopic("/pathfinder/admag").publish()
    distance = 100.0
  }

  fun runSetup() {
    timer.restart()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    ADStar.setGoalPosition(endPose.translation)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
    distance = robot.poseSubsystem.pose.translation.getDistance(endPose.translation)
    adMag = 1.0
    pidMag = 0.0
    if (distance < pidDistance) {
      adMag = (distance / pidDistance)
      pidMag = abs(1 - adMag)
    }
  }

  fun pathFind() {
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    val currentTime = timer.get()
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
      pidMag += pidIncRate
      if (pidMag > 1) {
        pidMag = 1.0
      }
      if (distance < tolerance) {
        atSetpoint = true
      } else if (distance < premoveDistance) {
        atPremoveDistance = true
      }
    } else {
      inPIDDistance = false
    }
    if (!atSetpoint) {
      if (ADStar.isNewPathAvailable) {
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

    val wrappedRotation = MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)
    rotation = thetaController.calculate(wrappedRotation)
    if (wrappedRotation == endPose.rotation.radians) {
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
    rotPub.set(wrappedRotation == endPose.rotation.radians)
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
  private val goal: Command
) :
  Command() {

  private val asPathfinder = command
  private var reefAlignCommand: Command = InstantCommand()
  private var usingReefAlign = false
  private var hasPremoved = false

  override fun initialize() {
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.schedule()
    asPathfinder.runSetup()
  }

  override fun execute() {
    if (!hasPremoved && asPathfinder.atPremoveDistance) {
      goal.schedule()
      hasPremoved = true
    }
    if (asPathfinder.atSetpoint && !usingReefAlign) {
      reefAlignCommand = SimpleReefAlign(robot.drive, robot.poseSubsystem)
      reefAlignCommand.schedule()
      usingReefAlign = true
    } else if (!usingReefAlign) {
      asPathfinder.pathFind()
    }
  }

  override fun isFinished(): Boolean {
    return usingReefAlign && reefAlignCommand.isFinished
  }
}
