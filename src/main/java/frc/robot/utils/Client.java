package frc.robot.utils;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

public class Client {
    private Socket socket;
    private PrintWriter out;
    private BufferedReader in;
    private DataInputStream dataIn;

    public Client(String host, int port) throws IOException {
        socket = new Socket(host, port);
        out = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()), true);
        dataIn = new DataInputStream(socket.getInputStream());
        System.out.println("Cliente conectado a " + host + ":" + port);
    }

    // ✅ Enviar una línea simple como espera el servidor (sin writeUTF)
    public void sendMessage(String message) {
        if (out != null) {
            out.print(message);
            out.flush();
        }
    }

    // ✅ Recibir una línea simple
    public String receiveMessage() throws IOException {
        dataIn = new DataInputStream(socket.getInputStream());
        byte[] buffer = new byte[200];
        dataIn.readFully(buffer);
        if (buffer != null) {
            String message = new String(buffer, StandardCharsets.UTF_8);
            System.out.println("Mensaje recibido: " + message);
        }
        return null;
    }

    public int[][] receivePath() throws IOException {
        String countMsg = receiveMessage();
        if (countMsg == null) throw new IOException("No se recibió tamaño del path");

        int count = Integer.parseInt(countMsg); // número de puntos
        int byteSize = Integer.parseInt(receiveMessage()); // tamaño total en bytes

        int[][] path = new int[count][2];

        byte[] buffer = new byte[byteSize];  // count * 2 * sizeof(int)
        dataIn.readFully(buffer);

        ByteBuffer wrapped = ByteBuffer.wrap(buffer);
        wrapped.order(ByteOrder.LITTLE_ENDIAN);

        for (int i = 0; i < count; i++) {
            int y = wrapped.getInt();
            int x = wrapped.getInt();
            path[i][0] = x;
            path[i][1] = y;
        }

        return path;
    }

    public void closeClient() throws IOException {
        if (socket != null) socket.close();
        if (out != null) out.close();
        if (in != null) in.close();
        if (dataIn != null) dataIn.close();
        System.out.println("Cliente cerrado.");
    }
}
