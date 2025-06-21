package frc.robot.utils;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class CoprocessorClient {

    private static final String SERVER_IP = "127.0.0.1";
    private static final int PORT = 27015;

    private Socket socket;
    private DataOutputStream out;
    private DataInputStream in;

    public CoprocessorClient() throws IOException {
        socket = new Socket(SERVER_IP, PORT);
        out = new DataOutputStream(socket.getOutputStream());
        in = new DataInputStream(socket.getInputStream());
        System.out.println("Conectado al coprocesador en " + SERVER_IP + ":" + PORT);
    }

    public void close() throws IOException {
        sendMessage("close");
        socket.close();
        System.out.println("Conexión con el coprocesador cerrada.");
    }

    private void sendMessage(String msg) throws IOException {
        byte[] data = msg.getBytes("UTF-8");
        ByteBuffer buffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
        buffer.putInt(data.length);
        out.write(buffer.array());
        out.write(data);
        out.flush();
        System.out.println("\033[32mEnviado: " + msg + " (" + data.length + " bytes)\033[37m");
    }

    private String receiveMessage() throws IOException {
        byte[] lenBytes = new byte[4];
        in.readFully(lenBytes);
        int len = ByteBuffer.wrap(lenBytes).order(ByteOrder.LITTLE_ENDIAN).getInt();
        byte[] data = new byte[len];
        in.readFully(data);
        String msg = new String(data, "UTF-8");
        System.out.println("\033[34m[Server] " + msg + " (" + len + " bytes)\033[37m");
        return msg;
    }

    public boolean testConnection() throws IOException {
        sendMessage("test");

        String response = receiveMessage();
        if (!"ok".equals(response)) {
            System.out.println("Respuesta inesperada del servidor.");
            return false;
        }

        sendMessage("ok");
        response = receiveMessage();

        if ("test ok".equals(response)) {
            System.out.println("Test completado con éxito.");
            return true;
        }

        return false;
    }

    public List<double[]> sendPathfind(double sx, double sy, double dx, double dy) throws IOException {
        sendMessage("pathfind");
        if (!"ok".equals(receiveMessage())) return List.of();
    
        sendMessage(String.valueOf(sx));
        if (!"ok".equals(receiveMessage())) return List.of();
    
        sendMessage(String.valueOf(sy));
        if (!"ok".equals(receiveMessage())) return List.of();
    
        sendMessage(String.valueOf(dx));
        if (!"ok".equals(receiveMessage())) return List.of();
    
        sendMessage(String.valueOf(dy));
        if (!"ok".equals(receiveMessage())) return List.of();
    
        List<double[]> path = new ArrayList<>();
    
        // Recibir pares (x, y) hasta que llegue "end"
        while (true) {
            String xStr = receiveMessage();
            if ("end".equals(xStr)) break;
    
            String yStr = receiveMessage();
            if ("end".equals(yStr)) break;
            
            if (xStr.equals("no path found")) {
                System.out.println("No se encontró un camino.");
                System.out.println(yStr);
                return List.of();
            }

            try {
                double x = Double.parseDouble(xStr);
                double y = Double.parseDouble(yStr);
                path.add(new double[]{x, y});
            } catch (NumberFormatException e) {
                System.err.println("Error al convertir coordenadas: " + xStr + ", " + yStr);
                break;
            }
        }
    
        // Enviar confirmación de recepción
        sendMessage("end");
    
        // Leer mensaje final (tiempo de ejecución u otro mensaje del servidor)
        String finalMessage = receiveMessage();
        System.out.println(finalMessage);
    
        return path;
    }
    
    
    
}
