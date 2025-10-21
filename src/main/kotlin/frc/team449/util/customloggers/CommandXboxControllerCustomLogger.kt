package frc.team449.util.customloggers

import edu.wpi.first.epilogue.CustomLoggerFor
import edu.wpi.first.epilogue.logging.ClassSpecificLogger
import edu.wpi.first.epilogue.logging.EpilogueBackend
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import kotlin.math.log

@CustomLoggerFor(CommandXboxController::class)
class CommandXboxControllerCustomLogger : ClassSpecificLogger<CommandXboxController>(CommandXboxController::class.java) {
  override fun update(backend: EpilogueBackend, controller: CommandXboxController) {
    backend.log("A", controller.a().asBoolean)
    backend.log("B", controller.b().asBoolean)
    backend.log("back", controller.back().asBoolean)
    backend.log("left bumper", controller.leftBumper().asBoolean)
    backend.log("left stick", controller.leftStick().asBoolean)
    backend.log("left trigger", controller.leftTriggerAxis)
    backend.log("left X", controller.leftX)
    backend.log("left Y", controller.leftY)
    backend.log("POV up", controller.povUp().asBoolean)
    backend.log("POV up right", controller.povUpRight().asBoolean)
    backend.log("POV right", controller.povRight().asBoolean)
    backend.log("POV down right", controller.povDownRight().asBoolean)
    backend.log("POV down", controller.povDown().asBoolean)
    backend.log("POV down left", controller.povDownLeft().asBoolean)
    backend.log("POV left", controller.povLeft().asBoolean)
    backend.log("POV up left", controller.povUpLeft().asBoolean)
    backend.log("right bumper", controller.leftBumper().asBoolean)
    backend.log("right stick", controller.rightStick().asBoolean)
    backend.log("right trigger", controller.rightTriggerAxis)
    backend.log("right X", controller.rightX)
    backend.log("right Y", controller.rightY)
    backend.log("start", controller.start().asBoolean)
    backend.log("X", controller.x().asBoolean)
    backend.log("Y", controller.y().asBoolean)
  }
}
