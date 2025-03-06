package frc.team449.subsystems.light

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.LEDPattern.GradientType
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier

/**
 * Controls an LED strip.
 * @param port The PWM port of the LED strip.
 * @param length The length of the LED strip.
 */

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private val lightStrip = AddressableLED(port)
  private val lightBuffer = AddressableLEDBuffer(length)

  val ledSpacing: Distance = Meters.of(1.0 / LightConstants.LED_PER_METER)

  init {
    lightStrip.setLength(lightBuffer.length)
    lightStrip.setData(lightBuffer)
    lightStrip.start()
  }

  fun rainbow(): Command {
    return this.run {
      val rainbow: LEDPattern = LEDPattern.rainbow(255, 128)
      val scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(
        MetersPerSecond.of(LightConstants.LED_TRANSLATION_SPEED),
        ledSpacing
      )

      scrollingRainbow.applyTo(lightBuffer)

//      PrintCommand("Applied Rainbow Pattern to LEDs.")
    }
  }

  fun solidColor(color: Color): Command {
    return this.run {
      val colorSolid: LEDPattern = LEDPattern.solid(color)

      colorSolid.applyTo(lightBuffer)

//      PrintCommand("Applied Solid Color '$color' to LEDs.").schedule()
    }
  }

  fun gradient(continuous: Boolean = true, speedMetersPerSecond: Double, vararg colors: Color): Command {
    return this.run {
      // continuous is optimal for scrolling, discontinuous for static
      val continuousness: GradientType = if (continuous) GradientType.kContinuous else GradientType.kDiscontinuous
      val gradient: LEDPattern = LEDPattern.gradient(continuousness, *colors)
      gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(speedMetersPerSecond), ledSpacing)

      gradient.applyTo(lightBuffer)

//    PrintCommand("Applied [continuous -> $continuous] gradient of colors [$colors] at speed $speedMetersPerSecond m/s to LEDs.").schedule()
    }
  }

  fun progressMask(maskValue: DoubleSupplier): Command {
    return this.run {
      val base: LEDPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kGreen)
      val maskAmount: LEDPattern = LEDPattern.progressMaskLayer(maskValue)
      val heightDisplay: LEDPattern = base.mask(maskAmount)

      heightDisplay.applyTo(lightBuffer)

//      PrintCommand("Applied value mask of $maskValue to LEDs.").schedule()
    }
  }

  override fun periodic() {
    lightStrip.setData(lightBuffer)
  }

  companion object {
    /** Create an LED strip controller using [LightConstants]. */
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
