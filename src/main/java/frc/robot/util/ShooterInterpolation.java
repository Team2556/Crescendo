package frc.robot.util;

import java.util.HashMap;
import java.util.Objects;

/**
 * Utility class to perform bilinear interpolation to determine the
 * optimal shooter angle given the XY position of the robot on the field.
 *
 * @author Christian
 */
public class ShooterInterpolation {
    private HashMap<Pair, Double> table = new HashMap<>();
    private final double minX = 0.00001, maxX = 8.99999, minY = 0.00001, maxY = 7.99999;

    public ShooterInterpolation() {
        populateTable();
    }

    public double calculate(double x, double y) {
        x = Math.max(minX, Math.min(x, maxX));
        y = Math.max(minY, Math.min(y, maxY));
        // Find the two closest X values
        double x1 = Math.floor(x);
        double x2 = Math.ceil(x);

        // Find the two closest Y values
        double y1 = Math.floor(y);
        y1 = y1 % 2 == 0 ? y1 : y1 - 1.0;
        double y2 = Math.ceil(y);
        y2 = y2 % 2 == 0 ? y2 : y2 + 1.0;

        // Retrieve the four nearest points
        double fQ11 = table.getOrDefault(new Pair(x1, y1), 0.0);
        double fQ12 = table.getOrDefault(new Pair(x1, y2), 0.0);
        double fQ21 = table.getOrDefault(new Pair(x2, y1), 0.0);
        double fQ22 = table.getOrDefault(new Pair(x2, y2), 0.0);

        // Perform bilinear interpolation
        double value = ((x2 - x) * (y2 - y) * fQ11 +
                (x - x1) * (y2 - y) * fQ21 +
                (x2 - x) * (y - y1) * fQ12 +
                (x - x1) * (y - y1) * fQ22) / ((x2 - x1) * (y2 - y1));
        return value;
    }

    private void populateTable() {
        //ToDo Replace with actual data points
        table.put(new Pair(0.0, 0.0), 80.0);
        table.put(new Pair(1.0, 0.0), 76.0);
        table.put(new Pair(2.0, 0.0), 70.0);
        table.put(new Pair(3.0, 0.0), 63.0);
        table.put(new Pair(4.0, 0.0), 52.0);
        table.put(new Pair(5.0, 0.0), 42.0);
        table.put(new Pair(6.0, 0.0), 30.0);
        table.put(new Pair(7.0, 0.0), 15.0);
        table.put(new Pair(8.0, 0.0), 7.0);
        table.put(new Pair(9.0, 0.0), 2.0);

        table.put(new Pair(0.0, 2.0), 82.0);
        table.put(new Pair(1.0, 2.0), 78.0);
        table.put(new Pair(2.0, 2.0), 72.0);
        table.put(new Pair(3.0, 2.0), 65.0);
        table.put(new Pair(4.0, 2.0), 54.0);
        table.put(new Pair(5.0, 2.0), 44.0);
        table.put(new Pair(6.0, 2.0), 32.0);
        table.put(new Pair(7.0, 2.0), 17.0);
        table.put(new Pair(8.0, 2.0), 9.0);
        table.put(new Pair(9.0, 2.0), 4.0);

        table.put(new Pair(0.0, 4.0), 84.0);
        table.put(new Pair(1.0, 4.0), 80.0);
        table.put(new Pair(2.0, 4.0), 74.0);
        table.put(new Pair(3.0, 4.0), 67.0);
        table.put(new Pair(4.0, 4.0), 56.0);
        table.put(new Pair(5.0, 4.0), 46.0);
        table.put(new Pair(6.0, 4.0), 34.0);
        table.put(new Pair(7.0, 4.0), 19.0);
        table.put(new Pair(8.0, 4.0), 11.0);
        table.put(new Pair(9.0, 4.0), 6.0);

        table.put(new Pair(0.0, 6.0), 82.0);
        table.put(new Pair(1.0, 6.0), 78.0);
        table.put(new Pair(2.0, 6.0), 72.0);
        table.put(new Pair(3.0, 6.0), 65.0);
        table.put(new Pair(4.0, 6.0), 54.0);
        table.put(new Pair(5.0, 6.0), 44.0);
        table.put(new Pair(6.0, 6.0), 32.0);
        table.put(new Pair(7.0, 6.0), 17.0);
        table.put(new Pair(8.0, 6.0), 9.0);
        table.put(new Pair(9.0, 6.0), 4.0);

        table.put(new Pair(0.0, 8.0), 80.0);
        table.put(new Pair(1.0, 8.0), 76.0);
        table.put(new Pair(2.0, 8.0), 70.0);
        table.put(new Pair(3.0, 8.0), 63.0);
        table.put(new Pair(4.0, 8.0), 52.0);
        table.put(new Pair(5.0, 8.0), 42.0);
        table.put(new Pair(6.0, 8.0), 30.0);
        table.put(new Pair(7.0, 8.0), 15.0);
        table.put(new Pair(8.0, 8.0), 7.0);
        table.put(new Pair(9.0, 8.0), 2.0);
    }

    public static class Pair {
        private final double x,y;
        public Pair(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            Pair pair = (Pair) obj;
            return Double.compare(pair.x, x) == 0 && Double.compare(pair.y, y) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y);
        }
    }
}