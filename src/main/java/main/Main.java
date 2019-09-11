package main;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.File;

public class Main extends Application {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) throws Exception {
        String os = System.getProperty("os.name").toLowerCase();
        FXMLLoader loader;
        if(os.equals("win"))
            loader = new FXMLLoader(new File("src\\main\\java\\main\\Gui.fxml").toURI().toURL());
        else
            loader = new FXMLLoader(new File("src/main/java/main/Gui.fxml").toURI().toURL());
        Scene scene = new Scene(loader.load());
        primaryStage.setScene(scene);
        primaryStage.show();
    }
}
