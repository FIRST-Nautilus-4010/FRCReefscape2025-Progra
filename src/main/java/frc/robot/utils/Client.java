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

    public Client(String host, int port) throws IOException {
        socket = new Socket(host, port);
        out = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()), true);
        in = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
        System.out.println("Cliente conectado a " + host + ":" + port);
    }

    // Enviar un mensaje, agregando '\n' para el servidor
    public void sendMessage(String message) {
        if (out != null) {
            out.print(message); // println agrega el '\n' automáticamente
            out.flush();
        }
    }

    // Recibir una línea completa (terminada en '\n')
    public String receiveMessage() throws IOException {
        String line = in.readLine(); // lee hasta '\n'
        if (line != null) {
            System.out.println("Mensaje recibido: " + line);
        }
        return line;
    }

    // (Tu método receivePath() puede quedarse igual, solo llamará a receiveMessage() que ya separa)
    public int[][] receivePath() throws IOException {
        String countMsg = receiveMessage();
        if (countMsg == null) throw new IOException("No se recibió tamaño del path");

        int count = Integer.parseInt(countMsg);
        int byteSize = Integer.parseInt(receiveMessage());

        int[][] path = new int[count][2];

        byte[] buffer = new byte[byteSize];
        int totalRead = 0;
        while (totalRead < byteSize) {
            int read = socket.getInputStream().read(buffer, totalRead, byteSize - totalRead);
            if (read == -1) throw new IOException("Stream cerrado prematuramente");
            totalRead += read;
        }

        ByteBuffer wrapped = ByteBuffer.wrap(buffer).order(ByteOrder.LITTLE_ENDIAN);

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
        System.out.println("Cliente cerrado.");
    }
}

