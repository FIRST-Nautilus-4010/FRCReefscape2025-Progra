package frc.robot.utils;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class CoprocessorClient {

    public static class Pair {
        public int first;
        public int second;

        public Pair(int first, int second) {
            this.first = first;
            this.second = second;
        }

        @Override
        public String toString() {
            return "(" + first + ", " + second + ")";
        }
    }

    private static final String SERVER_IP = "192.168.56.1";
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

    public List<Pair> sendPathfind(int sx, int sy, int dx, int dy) throws IOException {
        sendMessage("pathfind");
        if (!"ok".equals(receiveMessage())) return List.of();

        sendMessage(String.valueOf(sx));
        if (!"ok".equals(receiveMessage())) return List.of();

        sendMessage(String.valueOf(sy));
        if (!"ok".equals(receiveMessage())) return List.of();

        sendMessage(String.valueOf(dx));
        if (!"ok".equals(receiveMessage())) return List.of();

        sendMessage(String.valueOf(dy));
        String response = receiveMessage();

        if ("no path found".equals(response)) return List.of();

        int count = Integer.parseInt(response);
        int byteSize = Integer.parseInt(receiveMessage());

        byte[] buffer = new byte[byteSize];
        in.readFully(buffer);

        List<Pair> path = new ArrayList<>();
        ByteBuffer bb = ByteBuffer.wrap(buffer).order(ByteOrder.LITTLE_ENDIAN);
        for (int i = 0; i < count; i++) {
            int y = bb.getInt();
            int x = bb.getInt();
            path.add(new Pair(x, y));
        }

        sendMessage("end");
        System.out.println("[srv] " + receiveMessage());

        return path;
    }
}
