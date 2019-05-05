package main;

import gen.Trajectory;
import gen.Waypoint;
import javafx.fxml.FXML;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.control.cell.TextFieldListCell;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.util.StringConverter;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class Gui {
    @FXML
    public ChoiceBox<String> spline = null;
    public TextField dt = null;
    public TextField velocity = null;
    public TextField acceleration = null;
    public TextField jerk = null;
    public TextField width = null;
    public Slider tightness = null;

    public ListView<String> x = null;
    public ListView<String> y = null;
    public ListView<String> angle = null;
    public Button add = null;
    public Button delete = null;

    public Button gen = null;
    public Button save = null;
    public Button load = null;

    @FXML
    public void addPoint() {
        Stage prompt = new Stage();
        VBox root = new VBox();
        TextField x = new TextField();
        root.getChildren().add(new HBox(new Label("X Pos: "), x));
        TextField y = new TextField();
        root.getChildren().add(new HBox(new Label("Y Pos: "), y));
        TextField theta = new TextField();
        root.getChildren().add(new HBox(new Label("Angle: "), theta));
        Button confirm = new Button("OK");

        confirm.setOnAction((e) -> {
            try {
                this.x.getItems().add(Double.toString(Double.parseDouble(x.getText())));
                this.y.getItems().add(Double.toString(Double.parseDouble(y.getText())));
                this.angle.getItems().add(Double.toString(Double.parseDouble(theta.getText())));
                prompt.close();
            }catch (NumberFormatException n) {
                System.out.println("not a number!!!");
            }
        });
        x.setOnAction((e) -> y.requestFocus());
        x.setOnKeyPressed((ke) -> { if(y.getText().isEmpty() && ke.getCode().equals(KeyCode.BACK_SPACE)) prompt.close(); });
        y.setOnAction((e) -> theta.requestFocus());
        y.setOnKeyPressed((ke) -> { if(y.getText().isEmpty() && ke.getCode().equals(KeyCode.BACK_SPACE)) x.requestFocus(); });
        theta.setOnAction((e) -> confirm.fire());
        theta.setOnKeyPressed((ke) -> { if(theta.getText().isEmpty() && ke.getCode().equals(KeyCode.BACK_SPACE)) y.requestFocus(); });

        root.getChildren().add(confirm);
        root.setSpacing(10);
        root.setAlignment(Pos.CENTER);
        root.setPadding(new Insets(20));
        prompt.setScene(new Scene(root));
        prompt.show();
    }

    @FXML
    public void deletePoint() {
        int index = x.getSelectionModel().getSelectedIndices().get(0);
        if(index != -1) {
            x.getItems().remove(index);
            y.getItems().remove(index);
            angle.getItems().remove(index);
        }else {
            x.getItems().remove(x.getItems().size()-1);
            y.getItems().remove(y.getItems().size()-1);
            angle.getItems().remove(angle.getItems().size()-1);
        }
    }

    @FXML
    public void generate() {
        FileChooser chooser = new FileChooser();
        chooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Comma Separated Values", "*.csv"));
        chooser.setInitialFileName(getDateTimeString());
        File file = chooser.showSaveDialog(new Stage());

        try {
            Waypoint[] waypoints = new Waypoint[x.getItems().size()];
            for(int i = 0; i < waypoints.length; i++) {
                waypoints[i] = new Waypoint(Double.parseDouble(x.getItems().get(i)),
                        Double.parseDouble(y.getItems().get(i)),
                        Math.toRadians(Double.parseDouble(angle.getItems().get(i))));
            }
            Trajectory trajectory = new Trajectory(Trajectory.FitMethod.getMethod(spline.getValue()), 100000, tightness.getValue(),
                    Double.parseDouble(dt.getText()), Double.parseDouble(width.getText()),
                    Double.parseDouble(velocity.getText()), Double.parseDouble(acceleration.getText()), Double.parseDouble(jerk.getText()),
                    waypoints);
            trajectory.generate();

            CSVPrinter printer = new CSVPrinter(new FileWriter(file), CSVFormat.DEFAULT);
            printer.printRecord("time", "x", "y", "distance", "velocity", "acceleration", "jerk", "heading");
            for(Waypoint pt : trajectory.getPath()) {
                printer.printRecord(pt.time, pt.x, pt.y, pt.distanceFromStart, pt.velocity, pt.acceleration, pt.jerk, Math.toDegrees(pt.heading));
            }
            printer.close();
        }catch(NumberFormatException n) {
            System.out.println("not a number!!!");
        }catch (IOException io) {
            System.out.println("invalid file!!!");
        }
    }

    @FXML
    public void save() {
        FileChooser chooser = new FileChooser();
        chooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("javascript object notation", "*.json"));
        chooser.setInitialFileName(getDateTimeString());
        File file = chooser.showSaveDialog(new Stage());
        try (FileWriter writer = new FileWriter(file)) {
            JSONObject root = new JSONObject();
            root.put("spline", this.spline.getValue());
            root.put("dt", Double.parseDouble(dt.getText()));
            root.put("width", Double.parseDouble(width.getText()));
            root.put("velocity", Double.parseDouble(velocity.getText()));
            root.put("acceleration", Double.parseDouble(acceleration.getText()));
            root.put("jerk", Double.parseDouble(jerk.getText()));
            root.put("tightness", tightness.getValue());
            JSONArray waypoints = new JSONArray();
            for(int i = 0; i < x.getItems().size(); i++) {
                JSONArray point = new JSONArray();
                point.add(Double.parseDouble(x.getItems().get(i)));
                point.add(Double.parseDouble(y.getItems().get(i)));
                point.add(Double.parseDouble(angle.getItems().get(i)));
                waypoints.add(point);
            }
            root.put("waypoints", waypoints);
            writer.write(root.toJSONString());
            writer.flush();
        }catch (Exception e) {
            e.printStackTrace();
        }
    }

    @FXML
    public void load() {
        FileChooser chooser = new FileChooser();
        chooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("javascript object notation", "*.json"));
        File file = chooser.showOpenDialog(new Stage());
        JSONParser parser = new JSONParser();
        try (FileReader reader = new FileReader(file)) {
            JSONObject root = (JSONObject) parser.parse(reader);
            spline.setValue((String) root.get("spline"));
            dt.setText(Double.toString((double) root.get("dt")));
            velocity.setText(Double.toString((double) root.get("velocity")));
            acceleration.setText(Double.toString((double) root.get("acceleration")));
            jerk.setText(Double.toString((double) root.get("jerk")));
            width.setText(Double.toString((double) root.get("width")));
            tightness.setValue((double) root.get("tightness"));
            JSONArray waypoints = (JSONArray) root.get("waypoints");
            x.getItems().clear(); y.getItems().clear(); angle.getItems().clear();
            for (Object waypoint : waypoints) {
                JSONArray point = (JSONArray) waypoint;
                x.getItems().add(Double.toString((double) point.get(0)));
                y.getItems().add(Double.toString((double) point.get(1)));
                angle.getItems().add(Double.toString((double) point.get(2)));
            }
            parser.reset(reader);
        }catch (Exception e) {
            e.printStackTrace();
        }
    }

    private String getDateTimeString() {
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("MMddyyyy_HHmmss");
        LocalDateTime now = LocalDateTime.now();
        return dtf.format(now);
    }

    @FXML
    public void initialize() {
        spline.getItems().add("cubic hermite");
        spline.getItems().add("cubic bezier");
        spline.setValue("cubic hermite");

        x.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> {
            int index = x.getSelectionModel().getSelectedIndices().get(0);
            y.getSelectionModel().select(index);
            angle.getSelectionModel().select(index);
        });
        y.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> {
            int index = y.getSelectionModel().getSelectedIndices().get(0);
            x.getSelectionModel().select(index);
            angle.getSelectionModel().select(index);
        });
        angle.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> {
            int index = angle.getSelectionModel().getSelectedIndices().get(0);
            x.getSelectionModel().select(index);
            y.getSelectionModel().select(index);
        });

        StringConverter<String> converter = new StringConverter<String>() {
            @Override
            public String toString(String object) {
                return Double.toString(Double.parseDouble(object));
            }

            @Override
            public String fromString(String string) {
                return string;
            }
        };
        x.setCellFactory(TextFieldListCell.forListView(converter));
        y.setCellFactory(TextFieldListCell.forListView(converter));
        angle.setCellFactory(TextFieldListCell.forListView(converter));
    }
}
