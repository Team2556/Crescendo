package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PixyValues;

import static frc.robot.Constants.PixyValues;


public class PixycamSubsystem extends SubsystemBase {
    private final static PixycamSubsystem instance = getInstance();
    private static SerialPort m_arduinoPort;
    // private static int pixy_xVal = -1;
    private int pixy_center; // Distance of center of Note from center of Pixy's vision- should always be
    // less than 158
    private boolean pixy_positive; // Boolean to set if object is to left or right side of Pixy center
    private static int[] objCoord = { 0, 0 }; // Coordinates of the note from the Pixycam. Form {X-cord, Y-coord}
    private static String[] values;
    private static String valueX;
    private static String valueY;
    static double pixyHeight = PixyValues.pixy_Height;
    static double ringHeight = PixyValues.ring_Height;
    static double pixy_ff = PixyValues.ff;
    static double speed = PixyValues.turn_speed;

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
            valueX = values[0];
            valueY = values[1];
            SmartDashboard.putString("valueX", valueX);
            SmartDashboard.putString("valueY", valueY);
            objCoord[0] = Integer.valueOf(valueX);
            objCoord[1] = Integer.valueOf(valueY);  
            // objCoord[1] = Integer.valueOf(values[1]);
            // objCoord[0] = Integer.valueOf(values[0]);
        }

    }


    public int getXCoord(){
        readData();
        return objCoord[0];
    }

    public int getPixyCenter(){
        return getXCoord() - 158;
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
        if (Math.abs(pixy_center) > 8) { // Is pixy cam within 8 pixel range of center?
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
    //COME BACK TO THIS- Method to find distance between pixy and ring in mm
    public double distance_from_note(){
        double noteDistance = (pixy_ff* ringHeight* 152)/ (objCoord[0]* pixyHeight);
        return noteDistance;
    }

    public double calculateAngle(){
        double pixyRotation = objCoord[0]/ distance_from_note();
        double radiansPerSecond = (pixyRotation *(Math.PI/180))/ speed;
        if (Math.abs(radiansPerSecond) < 0.2) {
            radiansPerSecond = 0;
        }
        return radiansPerSecond;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance from note", distance_from_note());
        SmartDashboard.putNumber("pixyRotation", objCoord[0]/ distance_from_note());
        SmartDashboard.putNumber("radiansPerSecond", calculateAngle());
    }

    public static PixycamSubsystem getInstance() {
        if (instance == null)
            return new PixycamSubsystem();
        return instance;

    }
}