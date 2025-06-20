package frc.robot.utils;

import java.io.*;
import java.net.*;
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

    public static List<Pair> sendSimpleCmd(String cmd) {
        try (Socket socket = new Socket(SERVER_IP, PORT)) {
            socket.setSoTimeout(1000); // 1 segundo de timeout

            DataOutputStream out = new DataOutputStream(socket.getOutputStream());
            DataInputStream in = new DataInputStream(socket.getInputStream());

            System.out.println("Conectado al servidor");
            out.write(Arrays.copyOf(cmd.getBytes(), 200)); // padded to 200 bytes

            byte[] buffer = new byte[200];

            if ("test".equals(cmd)) {
                in.readFully(buffer);
                String response = new String(buffer).trim();
                System.out.println("Recibido del servidor: [" + response + "]");

                if (!"ok".equals(response)) {
                    System.out.println("Esperando confirmación del servidor...");
                    return List.of();
                }

                out.write(Arrays.copyOf("ok".getBytes(), 200));
                in.readFully(buffer);
                response = new String(buffer).trim();
                System.out.println("Recibido del servidor: [" + response + "]");

                if ("test ok".equals(response)) {
                    System.out.println("Prueba completada con éxito.");
                    return List.of(new Pair(1, 1));
                }
            } else {
                in.readFully(buffer);
                System.out.println("[srv] " + new String(buffer).trim());
            }

        } catch (IOException e) {
            System.err.println("Error en sendSimpleCmd: " + e.getMessage());
        }

        return List.of();
    }

    public static List<Pair> sendPathCmd(String cmd, int sx, int sy, int dx, int dy) {
        try (Socket socket = new Socket(SERVER_IP, PORT)) {
            socket.setSoTimeout(1000);

            DataOutputStream out = new DataOutputStream(socket.getOutputStream());
            DataInputStream in = new DataInputStream(socket.getInputStream());

            System.out.println("Conectado al servidor");
            out.write(Arrays.copyOf(cmd.getBytes(), 200));

            byte[] buffer = new byte[200];
            in.readFully(buffer);
            System.out.println("[srv] " + new String(buffer).trim());

            // Enviar coordenadas
            out.write(Arrays.copyOf(String.valueOf(sx).getBytes(), 200));
            in.readFully(buffer);
            System.out.println("[srv] " + new String(buffer).trim());

            out.write(Arrays.copyOf(String.valueOf(sy).getBytes(), 200));
            in.readFully(buffer);
            System.out.println("[srv] " + new String(buffer).trim());

            out.write(Arrays.copyOf(String.valueOf(dx).getBytes(), 200));
            in.readFully(buffer);
            System.out.println("[srv] " + new String(buffer).trim());

            out.write(Arrays.copyOf(String.valueOf(dy).getBytes(), 200));
            in.readFully(buffer);
            String response = new String(buffer).trim();
            System.out.println("[srv] " + response);

            if ("no path found".equals(response)) {
                return List.of();
            }

            int count = Integer.parseInt(response);
            in.readFully(buffer);
            int size = Integer.parseInt(new String(buffer).trim()); // opcional

            System.out.println("Recibiendo " + count + " pares (" + count * 8 + " bytes)");

            List<Pair> path = new ArrayList<>();
            for (int i = 0; i < count; i++) {
                int first = in.readInt();
                int second = in.readInt();
                path.add(new Pair(first, second));
            }

            out.write(Arrays.copyOf("end".getBytes(), 200));
            in.readFully(buffer);
            System.out.println("[srv] " + new String(buffer).trim());

            return path;

        } catch (IOException e) {
            System.err.println("Error en sendPathCmd: " + e.getMessage());
        }

        return List.of();
    }
}
