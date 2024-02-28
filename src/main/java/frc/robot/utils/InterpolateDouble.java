package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class InterpolateDouble {
    private InterpolatingDoubleTreeMap map;

    public InterpolateDouble(FileReader csvFile) {
        map = new InterpolatingDoubleTreeMap();
        try (BufferedReader csvReader = new BufferedReader(csvFile)) {
            String line = csvReader.readLine();
            while (line != null) {
                String[] vals = line.split(",");
                map.put(Double.parseDouble(vals[0]), Double.parseDouble(vals[1]));
            }
        } catch (IOException ex) {
            System.out.println(ex.toString());
        }
    }

    public double getValue(double key) {
        return map.get(key);
    }
}
