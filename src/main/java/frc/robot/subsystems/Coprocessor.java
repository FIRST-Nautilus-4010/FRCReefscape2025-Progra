package frc.robot.subsystems;

import java.io.IOException;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Client;

public class Coprocessor extends SubsystemBase{
    boolean isConnected = false;
    boolean closed = false;
    Client client;

    public Coprocessor() throws IOException {
        client = new Client("localhost", 27015);
    }

    public boolean isConnected() {
        return isConnected;
    }

    public void closeConnection() {
        try {
            client.sendMessage("exit");
            client.closeClient();
            isConnected = false;
            closed = true;
        } catch (Exception e) {
            System.err.println("Error al cerrar la conexión con el coprocesador: " + e.getMessage());
        }
    }

    public boolean isClosed() {
        return closed;
    }

    public int[][] getPathfind(int x, int y, int dx, int dy) throws IOException {
        if (!isConnected) {
            System.err.println("Coprocesador no conectado. No se puede enviar el comando de pathfinding.");
            return null; // Retorna un path vacío si no está conectado
        }
        client.sendMessage("pathfind");
        client.receiveMessage(); // Espera la respuesta del coprocesador

        client.sendMessage(String.valueOf(x));
        client.receiveMessage(); // Espera la respuesta del coprocesador
        client.sendMessage(String.valueOf(y));
        client.receiveMessage(); // Espera la respuesta del coprocesador
        client.sendMessage(String.valueOf(dx));
        client.receiveMessage(); // Espera la respuesta del coprocesador
        client.sendMessage(String.valueOf(dy));
        client.receiveMessage();



        int[][] path = client.receivePath();
        client.sendMessage("end"); // Finaliza la comunicación de pathfinding
        System.out.println(client.receiveMessage());
        return path; // Retorna el path recibido del coprocesador
        
    }

    @Override
    public void periodic() {
        if (!isConnected & !closed) {
            try {
                client.sendMessage("test");
                String response = client.receiveMessage();
                if (response.equals("ok")) {
                    client.sendMessage("ok");
                    response = client.receiveMessage();
                    if (response.equals("test ok")) {
                        System.out.println("Coprocesador conectado correctamente.");
                        isConnected = true;
                    } else {
                        System.out.println("No se recibió respuesta del coprocesador.");
                        System.out.println("respuesta esperada: test ok");
                        System.out.println("respuesta recibida: " + response);
                        isConnected = false;
                    }
                } else {
                    System.out.println("No se recibió respuesta del coprocesador.");
                    System.out.println("respuesta esperada: ok");
                    System.out.println("respuesta recibida: " + response);
                    isConnected = false;
                }
            } catch (Exception e) {
                System.err.println("Error al conectar al coprocesador: " + e.getMessage());
            }
        }

    }
}
