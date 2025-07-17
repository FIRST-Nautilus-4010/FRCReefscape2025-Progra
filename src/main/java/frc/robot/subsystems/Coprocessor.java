package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CoprocessorClient;

public class Coprocessor extends SubsystemBase {
    private CoprocessorClient client;
    private boolean isConnected = false;
    private boolean closed = false;
    private long lastAttempt = 0;
    private static final long RETRY_INTERVAL_MS = 3000;

    public Coprocessor() {
        // Deferred connection until the first attempt
    }

    public boolean isConnected() {
        return isConnected; // <-- Returns whether the coprocessor is connected.
    }

    public boolean isClosed() {
        return closed; // <-- Returns whether the connection to the coprocessor is closed.
    }

    public void closeConnection() {
        if (client != null) { // <-- Checks if the client is initialized.
            try {
                client.close(); // <-- Closes the client connection.
            } catch (IOException e) {
                e.printStackTrace(); // <-- Prints the stack trace if an error occurs while closing.
            }
        }
        closed = true; // <-- Marks the connection as closed.
        isConnected = false; // <-- Updates the connection status to disconnected.
    }

    public List<double[]> getPathfind(double x, double y, double dx, double dy) {
        if (client == null || closed) return List.of(); // <-- Returns an empty list if the client is null or the connection is closed.

        try {
            return client.sendPathfind(x, y, dx, dy); // <-- Sends pathfinding request to the coprocessor.
        } catch (IOException e) {
            System.err.println("Error requesting path: " + e.getMessage()); // <-- Logs an error message if the request fails.
            return List.of(); // <-- Returns an empty list in case of an error.
        }
    }

    @Override
    public void periodic() {
        long now = System.currentTimeMillis();

        if (!isConnected && !closed && now - lastAttempt > RETRY_INTERVAL_MS) {
            lastAttempt = now;

            try {
                if (client == null)
                    client = new CoprocessorClient();

                if (client.testConnection()) {
                    isConnected = true;
                    System.out.println("Coprocesador conectado exitosamente.");
                }
            } catch (IOException e) {
                System.err.println("Error al conectar con el coprocesador: " + e.getMessage());
            }
        }
    }
}
