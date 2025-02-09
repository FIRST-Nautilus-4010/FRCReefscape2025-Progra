package frc.robot.utils;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;

public class SaveData {

    private static final String FILE_NAME = "datos.dat";
    private static HashMap<String, Double> dataMap;

    static {
        dataMap = loadDataFromFile();
    }

    @SuppressWarnings("unchecked")
    private static HashMap<String, Double> loadDataFromFile() {
        HashMap<String, Double> map = new HashMap<>();
        File file = new File(FILE_NAME);
        if (file.exists()) {
            try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(file))) {
                map = (HashMap<String, Double>) ois.readObject();
            } catch (IOException | ClassNotFoundException e) {
                e.printStackTrace();
            }
        }
        return map;
    }

    private static void writeDataToFile() {
        try (ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(FILE_NAME))) {
            oos.writeObject(dataMap);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void saveData(String key, double value) {
        dataMap.put(key, value);
        writeDataToFile();
    }

    public static double readData(String key) {
        if (dataMap.containsKey(key)) {
            return dataMap.get(key);
        } else {
            throw new IllegalArgumentException("No se encontró dato para la clave: " + key);
        }
    }
}
