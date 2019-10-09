package io;

import gen.Trajectory;
import gen.Waypoint;
import org.apache.commons.io.FilenameUtils;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class JSON {

    public static void save(Trajectory traj, File file) {
        try (FileWriter writer = new FileWriter(file)) {
            JSONObject root = new JSONObject();
            root.put("spline", traj.method.toString());
            root.put("dt", traj.dt);
            root.put("width", traj.wheelBaseWidth);
            root.put("velocity", traj.maxVelocity);
            root.put("acceleration", traj.maxAcceleration);
            root.put("jerk", traj.maxJerk);
            root.put("tightness", traj.spline.tightness);
            JSONArray waypoints = new JSONArray();
            for (Waypoint wp : traj.spline.getControlPoints()) {
                JSONArray jp = new JSONArray();
                jp.add(wp.x);
                jp.add(wp.y);
                jp.add(wp.heading);
                waypoints.add(jp);
            }
            root.put("waypoints", waypoints);
            writer.write(root.toJSONString());
            writer.flush();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static Trajectory load(File file) {
        String ext = FilenameUtils.getExtension(file.getName());
        if (ext.toLowerCase().equals("json")) {
            throw new IllegalArgumentException("Not a json file");
        }
        Trajectory traj = null;
        try (FileReader reader = new FileReader(file)) {
            JSONParser parser = new JSONParser();
            JSONObject root = (JSONObject) parser.parse(reader);
            Trajectory.FitMethod method = Trajectory.FitMethod.findMethod((String) root.get("spline"));
            double dt = Double.parseDouble((String) root.get("dt"));
            double width = Double.parseDouble((String) root.get("width"));
            double velocity = Double.parseDouble((String) root.get("velocity"));
            double acceleration = Double.parseDouble((String) root.get("acceleration"));
            double jerk = Double.parseDouble((String) root.get("jerk"));
            double tightness = Double.parseDouble((String) root.get("tightness"));
            JSONArray wps = (JSONArray) root.get("waypoints");
            Waypoint[] waypoints = new Waypoint[wps.size()];
            for (int i = 0; i < wps.size(); i++) {
                JSONArray jp = (JSONArray) wps.get(i);
                double x = Double.parseDouble((String) jp.get(0));
                double y = Double.parseDouble((String) jp.get(1));
                double h = Double.parseDouble((String) jp.get(2));
                waypoints[i] = new Waypoint(x, y, h);
            }
            int rate = Trajectory.SampleRate.valueOf("HIGH").getRate();
            traj = new Trajectory(method, rate, tightness, dt, width, velocity, acceleration, jerk, waypoints);
        } catch (Exception e) {
            e.printStackTrace();
        }
        return traj;
    }

}
