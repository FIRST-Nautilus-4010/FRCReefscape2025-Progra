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
        // Conexión diferida hasta primer intento
    }

    public boolean isConnected() {
        return isConnected;
    }

    public boolean isClosed() {
        return closed;
    }

    public void closeConnection() {
        if (client != null) {
            try {
                client.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        closed = true;
        isConnected = false;
    }

    public List<double[]> getPathfind(double x, double y, double dx, double dy) {
        
        if (client == null || closed) return List.of();

        try {
            return client.sendPathfind(x, y, dx, dy);
        } catch (IOException e) {
            System.err.println("Error al pedir path: " + e.getMessage());
            return List.of();
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
