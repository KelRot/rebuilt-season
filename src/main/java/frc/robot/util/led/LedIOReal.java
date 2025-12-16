package frc.robot.util.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedIOReal implements LedIO {
    private final AddressableLED m_led;

    public LedIOReal(int port, int length) {
        m_led = new AddressableLED(port);
        m_led.setLength(length);
        m_led.start();
    }

    @Override
    public void updateInputs(LedIOInputs inputs) {
        inputs.connected = true;
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        m_led.setData(buffer);
    }
}
