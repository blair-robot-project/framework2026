package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.Commands.ScoreL4
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  private val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    { sample: SwerveSample -> robot.drive.followTrajectory(robot, sample) },
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  fun taxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }

  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("one l4 ")
    val forward = middleRoutine.trajectory("OneL4/1")
    val end = middleRoutine.trajectory("OneL4/2")

    middleRoutine.active().onTrue(
      Commands.sequence(
        forward.resetOdometry(),
        forward.cmd().alongWith(
          WaitCommand(0.5).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
          )
        )
      )
    )

    forward.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        robot.drive.driveStop()
      )

    )
    return middleRoutine
  }

  /**Ground Intake Autos**/

  private fun getScoreCommand(reefLevel: Int): (FieldConstants.ReefSide) -> Command {
    return when (reefLevel) {
      2 -> { side: FieldConstants.ReefSide -> scoreL2PivotDirectional(side) }
      3 -> { side: FieldConstants.ReefSide -> scoreL3PivotSideDirectional(side) }
      4 -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
      else -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
    }
  }

  private fun getPremoveCommand(reefLevel: Int): SuperstructureGoal.SuperstructureState {
    return when (reefLevel) {
      2 -> SuperstructureGoal.L2_PREMOVE_PIVOT
      3 -> SuperstructureGoal.L3_PREMOVE_PIVOT
      4 -> SuperstructureGoal.L4_PREMOVE_PIVOTT
      else -> SuperstructureGoal.L2_PREMOVE_PIVOT
    }
  }

  // pass in "l" or "r" for direction
  private fun ground3Point5(direction: String, reefLevel: IntArray): AutoRoutine {
    val routine = autoFactory.newRoutine("3.5 Ground 3L4 ${if (direction == "r") "Right" else "Left"}")
    val preloadScore = routine.trajectory("GroundThreeHalf/1$direction")
    val firstPickup = routine.trajectory("GroundThreeHalf/2$direction")
    val firstScore = routine.trajectory("GroundThreeHalf/3$direction")
    val secondPickup = routine.trajectory("GroundThreeHalf/4$direction")
    val secondScore = routine.trajectory("GroundThreeHalf/5$direction")
    val thirdPickup = routine.trajectory("GroundThreeHalf/6$direction")
    val thirdScore = routine.trajectory("GroundThreeHalf/7$direction") // give up on 4 piece

    val firstPickupTime = if (direction == "l") 3.0 else 2.8
    val secondPickupTime = 2.7 // same on both
    val missNearPickupTime = 3.0 // same on both

    val missNearPickup = routine.trajectory("GroundThreeHalf/failnear1$direction")
    val missNearScore = routine.trajectory("GroundThreeHalf/failnear2$direction")
    val missNearSecondPickup = routine.trajectory("GroundThreeHalf/failnear3$direction")
    val missNearSecondScore = routine.trajectory("GroundThreeHalf/failnear4$direction") // not possible to 3 on miss
    val missMidPickup = routine.trajectory("GroundThreeHalf/failmid1$direction")
    val missMidScore = routine.trajectory("GroundThreeHalf/failmid2$direction") // not possible to 2 on double miss

    routine.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[0]))
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[0]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intake()).withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),
        ConditionalCommand(
          Commands.sequence(
            firstScore.cmd().alongWith(
              WaitCommand(0.52).andThen(
                robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
              )
            )
          ),

          // if we miss picking up the first coral, do a backup routine
          Commands.sequence(
            missNearPickup.cmd().alongWith(
              robot.intake.outtakeL1().withTimeout(1.0)
                .andThen(intake())
            ).withTimeout(missNearPickupTime + AutoConstants.INTAKE_TIMEOUT),
            ConditionalCommand(
              missNearScore.cmd().alongWith(
                WaitCommand(0.52).andThen(
                  robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
                )
              ).andThen(robot.drive.driveStop()),
              missMidPickup.cmd().alongWith(robot.intake.outtakeL1().withTimeout(1.0).andThen(intake())) // 1.5 on double miss
            ) { robot.intake.coralDetected() || !RobotBase.isReal() }
          )

        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
      )
    )

    // backup routines
    missNearScore.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[1]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        missNearSecondPickup.cmd().alongWith(intake()),
      )
    )

    firstScore.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[1]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intake()).withTimeout(secondPickupTime + AutoConstants.INTAKE_TIMEOUT),
        robot.drive.driveStop(),
        ConditionalCommand(
          secondScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[2]))
            )
          ),
          missMidPickup.cmd().alongWith(robot.intake.outtakeL1().withTimeout(1.0).andThen(intake()))
        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
      )
    )

    secondScore.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[2]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        intake().alongWith(thirdPickup.cmd()),
        robot.drive.driveStop(),
      )
    )

    return routine
  }

  // three l4 starting from a side then the back two reefs then half
  fun rightGround3L4Half(): AutoRoutine {
    return ground3Point5("r", intArrayOf(4, 4, 4))
  }

  fun leftGround3L4Half(): AutoRoutine {
    return ground3Point5("l", intArrayOf(4, 4, 4))
  }

  // back l4 and then sides 2 l4
  private fun threeL4(direction: String): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4 ${if (direction == "r") "Right" else "Left"}")
    val preloadScore = middlesides.trajectory("middleSides/1$direction")
    val firstPickup = middlesides.trajectory("middleSides/2$direction")
    val firstPresagedScore = middlesides.trajectory("middleSides/3$direction")
    val secondPickup = middlesides.trajectory("middleSides/4$direction")
    val secondPresagedScore = middlesides.trajectory("middleSides/5$direction")
    val end = middlesides.trajectory("middleSides/end$direction")

    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        firstPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
          )
        )
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        secondPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
          )
        )
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }

  fun left3L4(): AutoRoutine {
    return threeL4("l")
  }

  fun right3L4(): AutoRoutine {
    return threeL4("r")
  }

  private fun groundBack2L4L2(direction: String, reefLevel: IntArray): AutoRoutine {
    val rightBack2l4l2 = autoFactory.newRoutine("2 l4 and l2 ${if (direction == "r") "Right" else "Left"}")
    val scorePreloadB = rightBack2l4l2.trajectory("TwoL4L2/1$direction")
    val pickupMiddle = rightBack2l4l2.trajectory("TwoL4L2/2$direction")
    val scoreMiddleA = rightBack2l4l2.trajectory("TwoL4L2/3$direction")
    val pickupLeft = rightBack2l4l2.trajectory("TwoL4L2/4$direction")
    val scoreRightB = rightBack2l4l2.trajectory("TwoL4L2/5$direction")
    val pickupRight = rightBack2l4l2.trajectory("TwoL4L2/6$direction")
    val scoreLeftA = rightBack2l4l2.trajectory("TwoL4L2/7$direction")
    val end = rightBack2l4l2.trajectory("TwoL4L2/end$direction")

    val firstPickupTime = 3.0 // same on both

    val missMidPickup = rightBack2l4l2.trajectory("TwoL4L2/failmid1$direction")
    val missMidScore = rightBack2l4l2.trajectory("TwoL4L2/failmid2$direction")
    val missMidSecondPickup = rightBack2l4l2.trajectory("TwoL4L2/failmid3$direction")
    val missMidSecondScore = rightBack2l4l2.trajectory("TwoL4L2/failmid4$direction") // second score impossible on miss

    rightBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(robot.intake.stop()),
        scorePreloadB.cmd().alongWith(
          robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[0]))
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[0]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        pickupMiddle.cmd().alongWith(intake()).withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),
        ConditionalCommand(
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
            )
          ),
          Commands.sequence(
            missMidPickup.cmd().alongWith(robot.intake.outtakeL1().withTimeout(1.0).andThen(intake())),
            missMidScore.cmd().alongWith(
              WaitCommand(0.52).andThen(
                robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[1]))
              )
            )
          )
        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
      )
    )

    // backup routines
    missMidScore.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[1]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        missMidSecondPickup.cmd().alongWith(intake()),
//        missMidSecondScore.cmd().alongWith(
//          WaitCommand(0.52).andThen(
//            robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[2]))
//          )
//        ),
//        getScoreCommand(reefLevel[2]).invoke(if(direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        getScoreCommand(reefLevel[1]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        pickupLeft.cmd().alongWith(intake()),
        scoreRightB.cmd().alongWith(
          WaitCommand(0.74).andThen(
            robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[2]))
          )
        )
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          getScoreCommand(reefLevel[2]).invoke(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
          pickupRight.cmd().alongWith(intake()),
          scoreLeftA.cmd().alongWith(
            WaitCommand(0.68).andThen(
              robot.superstructureManager.requestGoal(getPremoveCommand(reefLevel[3]))
            )
          )
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          getScoreCommand(reefLevel[3]).invoke(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        )
      )

    return rightBack2l4l2
  }

  fun rightGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("r", intArrayOf(4, 4, 2, 2))
  }

  fun leftGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("l", intArrayOf(4, 4, 2, 2))
  }

  fun noAlignLeftBack2L4l2(): AutoRoutine {
    val leftBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = leftBack2l4l2.trajectory("noAlignTwoL4L2/1l")
    val pickupMiddle = leftBack2l4l2.trajectory("noAlignTwoL4L2/2l")
    val scoreMiddleA = leftBack2l4l2.trajectory("noAlignTwoL4L2/3l")
    val pickupLeft = leftBack2l4l2.trajectory("noAlignTwoL4L2/4l")
    val scoreRightB = leftBack2l4l2.trajectory("noAlignTwoL4L2/5l")
    val pickupRight = leftBack2l4l2.trajectory("noAlignTwoL4L2/6l")
    val scoreLeftA = leftBack2l4l2.trajectory("noAlignTwoL4L2/7l")
    val end = leftBack2l4l2.trajectory("noAlignTwoL4L2/endl")

    leftBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(
          robot.intake.stop()
        ),
        scorePreloadB.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        scoreL4PivotSide(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        scoreMiddleA.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
          )
        )
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        scoreL4PivotSide(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupLeft.cmd().alongWith(intake()),
        robot.drive.driveStop(),
        scoreRightB.cmd().alongWith(
          WaitCommand(0.74).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
          )
        )
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          scoreL2PivotSide(),
          pickupRight.cmd().alongWith(intake()),
          robot.drive.driveStop(),
          scoreLeftA.cmd().alongWith(
            WaitCommand(0.68).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE_PIVOT)
            )
          )
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          scoreL2PivotSide(),
          end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
          robot.drive.driveStop(),

        )
      )

    return leftBack2l4l2
  }

  fun americanRoutineOptimal(): AutoRoutine {
    val optimalAmerican = autoFactory.newRoutine("opt Ameriacn")
    val l4ATraj = optimalAmerican.trajectory("GroundThreeHalf/L4A (I)")
    val l4BTraj = optimalAmerican.trajectory("GroundThreeHalf/l4B")
    val loli1Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 1")
    val loli2Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 2")
    val l3Traj = optimalAmerican.trajectory("GroundThreeHalf/l3B")
    val loli3Traj = optimalAmerican.trajectory("GroundThreeHalf/Loli 3")

    optimalAmerican.active().onTrue(
      l4ATraj.resetOdometry().andThen(l4ATraj.cmd())

    )

    l4ATraj.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.LEFT),
        loli1Traj.cmd().alongWith(intake()),
        l4BTraj.cmd(),
        scoreL4PivotSideDirectional(FieldConstants.ReefSide.RIGHT),
        intake().alongWith(loli2Traj.cmd()),
        l3Traj.cmd().andThen(scoreL2PivotDirectional(FieldConstants.ReefSide.RIGHT)),
        loli3Traj.cmd().alongWith(intake())

      )
    )

    return optimalAmerican
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Right 3.5 L4 Back & Sides", this::rightGround3L4Half)
    autoChooser.addRoutine("Left 3.5 L4 Back & Sides", this::leftGround3L4Half)

    autoChooser.addRoutine("Right 2L4 2L2 Back", this::rightGroundBack2L4L2)
    autoChooser.addRoutine("Left 2L4 2L2 Back", this::leftGroundBack2L4L2)
    autoChooser.addRoutine("Left 2L4 2L2 Back No Align", this::noAlignLeftBack2L4l2)

    autoChooser.addRoutine("Left 3 L4 Middle & Sides", this::left3L4)
    autoChooser.addRoutine("Right 3 L4 Middle & Sides", this::right3L4)

    autoChooser.addRoutine("Taxi", this::taxi)

    autoChooser.addRoutine("Center 1 L4", this::middleRoutine)
  }

  private fun scoreL4PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() || !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  private fun scoreL4PivotSide(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() || !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  private fun scoreL3PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      .alongWith(
        // robot.intake.outtakeAlgae(),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() || !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  private fun scoreL2PivotDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() || !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  private fun scoreL2PivotSide(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() || !RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  private fun premoveIntake(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }

  private fun intake(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() || !RobotBase.isReal() }
      )
      .andThen(
        robot.intake.stop()
      )
  }
}
