package io;

import gen.Trajectory;
import gen.Vector;
import gen.Waypoint;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.csv.CSVRecord;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;

public class CSV {

    public static Waypoint[] importCSV(File csvData) {
        try {
            CSVParser parser = CSVParser.parse(csvData, StandardCharsets.UTF_8, CSVFormat.DEFAULT);
            List<CSVRecord> records = parser.getRecords();
            Map<String, Integer> headerMap = parser.getHeaderMap();
            int size = records.get(0).size();
            Waypoint[] waypoints = new Waypoint[size];
            int i = 0;
            for (CSVRecord record : records) {
                waypoints[i] = new Waypoint(getHead(record, "x", headerMap),
                                            getHead(record, "y", headerMap),
                                            getHead(record, "heading", headerMap));
                waypoints[i].acceleration = getHead(record, "acceleration", headerMap);
                waypoints[i].jerk = getHead(record, "jerk", headerMap);
                waypoints[i].time = getHead(record, "time", headerMap);
                waypoints[i].distanceFromStart = getHead(record, "distance", headerMap);
                waypoints[i].velocity = getHead(record, "velocity", headerMap);
                i++;
            }
            parser.close();
            return waypoints;
        } catch (IOException io) {
            System.out.print("invalid file");
        }
        return null;
    }

    public static void exportCSV(File file, Trajectory trajectory) {
        try {
            CSVPrinter printer = new CSVPrinter(new FileWriter(file), CSVFormat.DEFAULT);
            printer.printRecord("time", "x", "y", "distance", "velocity", "acceleration", "jerk", "heading");
            for (Waypoint pt : trajectory.getPoints()) {
                printer.printRecord(pt.time, pt.x, pt.y, pt.distanceFromStart, pt.velocity, pt.acceleration, pt.jerk, Math.toDegrees(pt.heading));
            }
            printer.close();
        } catch (IOException io) {
            System.out.println("invalid file!");
        }
    }

    private static double getHead(CSVRecord record, String header, Map<String, Integer> headerMap) {
        int key = headerMap.get(header);
        if (record.get(key) == null) {
            return Double.NaN;
        } else {
            return Double.parseDouble(record.get(key));
        }
    }

}
