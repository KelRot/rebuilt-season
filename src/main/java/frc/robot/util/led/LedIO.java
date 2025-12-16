package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
    @AutoLog
    public static class LedIOInputs {
        public boolean connected = true;
        public String activePatternName = "";
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(LedIOInputs inputs) {}

    /** Sets the data to the LED strip. */
    default void setData(AddressableLEDBuffer buffer) {}
}
