package frc.robot.subsystems;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final static Vision instance = getInstance();
  private static SerialPort m_arduinoPort;
  // private static int pixy_xVal = -1;
  private int pixy_center;
  boolean pixy_positive;
  private int objCoord[] = { 0, 0 };
  private String values[];

  public Vision() {
    // table = NetworkTableInstance.getDefault().getTable("limelight");

    /*
     * Instantiating arduino port as SerialPort, trying every instantiation
     * with kUSB, kUSB1, and kUSB2. We used try-catch to see which instantiation
     * failed
     */
    // int[] objCoord = {1, -1};
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

  public void readData() {

    String arduinoData = m_arduinoPort.readString();
    SmartDashboard.putString("data", arduinoData);
    if (arduinoData.contains("A")) {  //BANG -Shuman
      values = arduinoData.split("A");
      SmartDashboard.putString("valueX", values[0]);
      SmartDashboard.putString("valueY", values[1]);
      objCoord[1] = Integer.parseInt(values[1]);
      objCoord[0] = Integer.parseInt(values[0]);
    }

  }

  public String[] getPixyValue() {
    this.readData();
    return values;
  }

  // public double getPixySetPointValue() {
  // return pixy_SetPointVal;
  // }

  public double getError() {
    pixy_center = objCoord[0] - 158;
    if (pixy_center > 0) {
      pixy_positive = false;
    } else {
      pixy_positive = true;
    }
    return pixy_center;
  }

  public void pixy_range() {
    if ((-8 > pixy_center) || (pixy_center > 8)) { // Is pixy cam roughly centered on the note
      getError();
      if (pixy_positive = false) {
        // setmotors counter clockwise at .2 power
      } else {
        // set motors counterclockwise
      }
    } else {
      // doNothing
    }
  }

  // public double getAngleX() {
  // return table.getEntry("tx").getDouble(0);
  // }

  // public double getAngleY() {
  // return table.getEntry("ty").getDouble(0);
  // }

  // public double getArea() {
  // return table.getEntry("ta").getDouble(0);
  // }

  // public double getSkew() {
  // return table.getEntry("ts").getDouble(0);
  // }

  // public double getDist() {
  // return (2.496 - 0.991) / Math.tan((getAngleY() * Math.PI) / 180);
  // }

  public static Vision getInstance() {
    if (instance == null)
      return new Vision();
    return instance;

  }

  // public void setDefaultCommand(Pixy pixy) {
  // throw new UnsupportedOperationException("Unimplemented method
  // 'setDefaultCommand'");
  // }
}
