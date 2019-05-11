package followers;
import gen.Waypoint;
import org.apache.commons.csv.*;

import java.nio.charset.Charset;
import java.util.*;
import java.io.*;

class Path {
    private Waypoint[] targetPoints;
    private Map<String, Integer> headerMap;
    private List<CSVRecord> records;
    private double dt;

    Path(File csvData) {
        targetPoints = extractData(csvData);
        this.dt = calculateDt();
    }

    public double getDt() {
        return dt;
    }

    private double calculateDt() {
        double t0 = getHead(records.get(0), "time");
        double t1 = getHead(records.get(1), "time");
        return t1 - t0;
    }

    private Waypoint[] extractData(File csvData) {
        try {
            CSVParser parser = CSVParser.parse(csvData, Charset.forName("UTF-8"), CSVFormat.EXCEL);
            records = parser.getRecords();
            headerMap = parser.getHeaderMap();
            int size = records.get(0).size();
            Waypoint[] waypoints = new Waypoint[size];
            int i = 0;
            for (CSVRecord record : records) {
                waypoints[i] = new Waypoint(getHead(record, "x"), getHead(record, "y"));
                waypoints[i].acceleration = getHead(record, "acceleration");
                waypoints[i].jerk = getHead(record, "jerk");
                waypoints[i].heading = getHead(record, "heading");
                waypoints[i].time = getHead(record, "time");
                waypoints[i].distanceFromStart = getHead(record, "distance");
                waypoints[i].velocity = getHead(record, "velocity");
                i++;
            }
            parser.close();
            return waypoints;
        } catch (IOException ex) {
            System.out.print("You messed up");
        }
        return null;
    }

    Waypoint[] getTargetPoints() {
        return targetPoints;
    }

    private double getHead(CSVRecord record, String header) {
        int key = headerMap.get(header);
        if (record.get(key) == null) {
            return Double.NaN;
        } else {
            return Double.parseDouble(record.get(key));
        }
    }

}
