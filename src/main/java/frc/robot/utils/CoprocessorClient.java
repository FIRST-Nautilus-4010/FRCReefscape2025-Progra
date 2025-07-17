package frc.robot.utils;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class CoprocessorClient {

    private static final String SERVER_IP = "127.0.0.1"; // <-- IP address of the coprocessor server.
    private static final int PORT = 27015; // <-- Port number for the coprocessor connection.

    private Socket socket; // <-- Socket for communication with the coprocessor.
    private DataOutputStream out; // <-- Output stream for sending data to the coprocessor.
    private DataInputStream in; // <-- Input stream for receiving data from the coprocessor.

    public CoprocessorClient() throws IOException {
        // Initializes the connection to the coprocessor server.
        socket = new Socket(SERVER_IP, PORT); // <-- Creates a socket connection to the server.
        out = new DataOutputStream(socket.getOutputStream()); // <-- Sets up the output stream.
        in = new DataInputStream(socket.getInputStream()); // <-- Sets up the input stream.
        System.out.println("Connected to the coprocessor at " + SERVER_IP + ":" + PORT); // <-- Logs the connection status.
    }

    public void close() throws IOException {
        // Closes the connection to the coprocessor.
        sendMessage("close"); // <-- Sends a "close" message to the coprocessor.
        socket.close(); // <-- Closes the socket connection.
        System.out.println("Connection to the coprocessor closed."); // <-- Logs the disconnection status.
    }

    private void sendMessage(String msg) throws IOException {
        // Sends a message to the coprocessor.
        byte[] data = msg.getBytes("UTF-8"); // <-- Converts the message to a byte array using UTF-8 encoding.
        ByteBuffer buffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN); // <-- Prepares a buffer for the message length.
        buffer.putInt(data.length); // <-- Writes the length of the message to the buffer.
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
