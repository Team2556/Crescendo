package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PixycamSubsystem extends SubsystemBase {
  private final static PixycamSubsystem instance = getInstance();
  private static SerialPort m_arduinoPort;
  // private static int pixy_xVal = -1;
  private int pixy_center; // Distance of center of Note from center of Pixy's vision- should always be
                           // less than 158
  boolean pixy_positive; // Boolean to set if object is to left or right side of Pixy center
  private int objCoord[] = { 0, 0 }; // Coordinates of the note from the Pixycam. Form {X-cord, Y-coord}
  private String values[];

  public PixycamSubsystem() {

    /*
     * Instantiating arduino port as SerialPort, trying every instantiation
     * with kUSB, kUSB1, and kUSB2. We used try-catch to see which instantiation
     * failed
     */

    try {
      m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB);
      System.out.println("Connected to arduino in vision!");
    } catch (Exception e) {
      System.out.println("kUSB failed");

      try {
        m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB1);
        System.out.println("Connected to arduino in vision!");
      } catch (Exception e1) {
        System.out.println("kUSB1 failed");

        try {
          m_arduinoPort = new SerialPort(115200, SerialPort.Port.kUSB2);
          System.out.println("Connected to arduino in vision!");
        } catch (Exception e2) {
          System.out.println("kUSB2 failed");
        }
      }
    }
  }

  /* Function to parse data from the Arduino to RoboRIO in an array form. */
  public void readData() {

    String arduinoData = m_arduinoPort.readString();
    SmartDashboard.putString("data", arduinoData);
    if (arduinoData.contains("A")) { // BANG -Shuman 2024
      values = arduinoData.split("A");
      SmartDashboard.putString("valueX", values[0]);
      SmartDashboard.putString("valueY", values[1]);
      objCoord[1] = Integer.parseInt(values[1]);
      objCoord[0] = Integer.parseInt(values[0]);
    }

  }

  // Function to convert readData to a usable return type
  public String[] getPixyValue() {
    this.readData();
    return values;
  }

  // Function to get distance of Note from center of pixy's vision.
  // WIP- Needs to be added to a drive function
  public double getError() {
    pixy_center = objCoord[0] - 158;
    if (pixy_center > 0) {
      pixy_positive = false; // Note is to the left of Pixy center
    } else {
      pixy_positive = true; // Note is to right of Pixy center
    }
    return pixy_center;
  }
//Swerve implementable Pixy method
  public void pixy_range() {
    if ((-8 > pixy_center) || (pixy_center > 8)) { // Is pixy cam within 8 pixel range of center?
      getError(); //If not, check how far off
      if (pixy_positive = false) {
        // Turn robot to left
      } else {
        // turn motors right
      }
    } else {
      // doNothing
    }
  }

  public static PixycamSubsystem getInstance() {
    if (instance == null)
      return new PixycamSubsystem();
    return instance;

  }

  // public void setDefaultCommand(Pixy pixy) {
  // throw new UnsupportedOperationException("Unimplemented method
  // 'setDefaultCommand'");
  // }
}
