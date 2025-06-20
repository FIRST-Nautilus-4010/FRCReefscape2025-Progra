package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CoprocessorClient;
import frc.robot.utils.CoprocessorClient.Pair;

public class Coprocessor extends SubsystemBase {
    private boolean isConnected = false;
    private boolean closed = false;
    private long lastAttempt = 0;
    private static final long RETRY_INTERVAL_MS = 3000;

    public Coprocessor(){
        // Constructor vacío
    }

    public boolean isConnected() {
        return isConnected;
    }

    public void closeConnection() {
        closed = true;
        isConnected = false;
    }

    public boolean isClosed() {
        return closed;
    }

    public List<Pair> getPathfind(int x, int y, int dx, int dy) {
        return CoprocessorClient.sendPathCmd("pathfind", x, y, dx, dy);
    }

    @Override
    public void periodic() {
        long now = System.currentTimeMillis();
        if (!isConnected && !closed && now - lastAttempt > RETRY_INTERVAL_MS) {
            lastAttempt = now;
            List<Pair> test = CoprocessorClient.sendSimpleCmd("test");

            if (!test.isEmpty()) {
                isConnected = true;
                System.out.println("Coprocesador conectado exitosamente.");
            } else {
                System.err.println("Error al conectar con el coprocesador.");
            }
        }
    }
}
