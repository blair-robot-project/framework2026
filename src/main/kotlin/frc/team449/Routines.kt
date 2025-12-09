package frc.team449

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.commands.AutoPoseToPose

object Routines {

  private val autoFactory = AutoFactory(
    Robot.poseSubsystem::pose,
    Robot.poseSubsystem::resetOdometry,
    { sample: SwerveSample -> Robot.drive.set(AutoPoseToPose.calculate(Robot.poseSubsystem.pose, sample.pose)) },
    true,
    Robot.drive
  )

  private val autoChooser = AutoChooser()

  init {
    // Adds Auto Selection into Smart Dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser)
    // While in Autonomous Period, run the selected auto until autos are over, then cancel command.
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler())
  }

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  fun addRoutine(name: String, autoRoutine: AutoRoutine) {
    autoChooser.addRoutine(name, { autoRoutine })
  }
}
