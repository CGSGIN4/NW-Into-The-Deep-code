package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.math.curve;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class parser {
    private static final String COMMA_DELIMITER = ",";
    private static final String SEMICOLON_DELIMITER = ";";
    String FILENAME = "ITD_RED_YELLOW";
    String PATH = String.format("%s/FIRST/points/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME);
    BufferedReader reader;

    public parser(){
        try {
            File file = new File(String.format("%s/FIRST/points/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME));
            file.createNewFile();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
            reader = new BufferedReader(new FileReader(PATH));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    public parser(String filename){
        this.FILENAME = filename;
        this.PATH = String.format("%s/FIRST/points/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME);
        try {
            File file = new File(String.format("%s/FIRST/points/%s.csv", Environment.getExternalStorageDirectory().getPath(), FILENAME));
            file.createNewFile();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
            reader = new BufferedReader(new FileReader(PATH));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    public curve[] getCurves() throws IOException {
        List<curve> curves = new ArrayList<>();

        try (Scanner scanner = new Scanner(new File(PATH))) {
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                if (!line.isEmpty())
                    curves.add(getCurveFromLine(line));
            }
        }

        /*
        dataStorage.DSTelemetry.addData("point", points.toArray());
        dataStorage.DSTelemetry.update();
        */

        /*
        dataStorage.DSTelemetry.addData("line", line);
        dataStorage.DSTelemetry.update();
        while ((line = reader.readLine()) != null){
            dataStorage.DSTelemetry.addData("line", line);
            dataStorage.DSTelemetry.update();
            line = line.trim().replace("(", "").replace(")", "");
            String[] parts = line.split(",");
            if (parts.length == 2){
                double x = Double.parseDouble(parts[0].trim());
                double y = Double.parseDouble(parts[1].trim());
                points.add(new Vector2d(x, y));
            }
        }
         */
        return curves.toArray(new curve[0]);
    }

    private curve getCurveFromLine(String line) {
        try (Scanner rowScanner = new Scanner(line)) {
            rowScanner.useDelimiter(SEMICOLON_DELIMITER);
            Vector2d[] nodes = new Vector2d[4];
            for (int i = 0; i < 4; i++) {
                String str = rowScanner.next();
                str = str.trim().replace(")", "").replace("(", "");
                String[] parts = str.split(COMMA_DELIMITER);
                double x = Double.parseDouble(parts[0].trim());
                double y = Double.parseDouble(parts[1].trim());
                nodes[i] = new Vector2d(x, y);
                dataStorage.DSTelemetry.addData(""+ i, nodes[i]);
                dataStorage.DSTelemetry.update();
            }

            ArrayList<Double> turn_points = new ArrayList<>();

            String value;
            while (rowScanner.hasNext() && (value = rowScanner.next()) != null){
                turn_points.add(Double.parseDouble(value));
            }

            double[] turn_starts = new double[turn_points.toArray().length / 2];
            double[] turn_ends = new double[turn_points.toArray().length / 2];
            for (int i = 0; i < turn_points.toArray().length / 2; i++)
                turn_starts[i] = turn_points.get(i);
            for (int j = 0, i = turn_points.toArray().length / 2; i < turn_points.toArray().length; i++, j++)
                turn_ends[j] = turn_points.get(i);

            curve curve;
            try {
                curve = new curve(nodes, 50);
            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            for (double t : turn_starts) {
                curve.turn_starts.add(t);
            }

            for (double t : turn_ends) {
                curve.turn_ends.add(t);
            }
            return curve;
        }
    }
}
