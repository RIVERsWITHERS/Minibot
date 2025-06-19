package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arduino extends SubsystemBase {
    private SerialPort arduinoPort;

    public Arduino() {
        try {
            arduinoPort = new SerialPort(9600, SerialPort.Port.kUSB); // Try kUSB1 if needed
        } catch (Exception e) {
            System.out.println("Failed to connect to Arduino: " + e.getMessage());
        }
    }

    @Override
    public void periodic() {
        if (arduinoPort != null && arduinoPort.getBytesReceived() > 0) {
            String message = arduinoPort.readString();
            System.out.println("From Arduino: " + message);
        }
    }
}
