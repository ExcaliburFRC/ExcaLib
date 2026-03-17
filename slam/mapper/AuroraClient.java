package frc.excalib.slam.mapper;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;

public class AuroraClient {
    private ServerSocket serverSocket;
    private Thread serverThread;
    private volatile boolean running = true;

    // Pose Data (volatile for thread safety)
    private volatile float x = 0;
    private volatile float y = 0;
    private volatile float z = 0;
    private volatile float roll = 0;
    private volatile float pitch = 0;
    private volatile float yaw = 0;

    // Coral positions
    private volatile List<Translation2d> corals = List.of();

    // JSON POJOs
    public static class Pose {
        public float x;
        public float y;
        public float z;
        public float roll;
        public float pitch;
        public float yaw;
    }

    public static class VisionMessage {
        public Pose pose;
        public List<Translation2d> corals;
    }

    public AuroraClient(int port) {
        serverThread = new Thread(() -> {
            try {
                serverSocket = new ServerSocket(port);
                DriverStation.reportWarning("Localization server started on port " + port, false);

                ObjectMapper mapper = new ObjectMapper();
                JsonFactory factory = new JsonFactory();

                while (running) {
                    try (Socket clientSocket = serverSocket.accept();
                         InputStream in = clientSocket.getInputStream();
                         JsonParser parser = factory.createParser(in)) {

                        DriverStation.reportWarning("Localization client connected!", false);

                        // Continuously parse incoming JSON objects
                        while (running && !clientSocket.isClosed()) {
                            try {
                                VisionMessage msg = mapper.readValue(parser, VisionMessage.class);

                                if (msg.pose != null) {
                                    x = msg.pose.x;
                                    y = msg.pose.y;
                                    z = msg.pose.z;
                                    roll = msg.pose.roll;
                                    pitch = msg.pose.pitch;
                                    yaw = msg.pose.yaw;
                                }

                                if (msg.corals != null) {
                                    corals = msg.corals;
                                }

                            } catch (IOException e) {
                                DriverStation.reportError("Error parsing localization JSON: " + e.getMessage(), false);
                                break; // Exit client loop on parse error
                            }
                        }
                    } catch (IOException e) {
                        DriverStation.reportError("Client connection error: " + e.getMessage(), false);
                    }
                }
            } catch (IOException e) {
                DriverStation.reportError("Localization server error: " + e.getMessage(), false);
            }
        });

        serverThread.setDaemon(true);
        serverThread.start();
    }

    // Getter methods for retrieving pose data
    public float getX() { return x; }
    public float getY() { return y; }
    public float getZ() { return z; }
    public float getRoll() { return roll; }
    public float getPitch() { return pitch; }
    public float getYaw() { return yaw; }
    public List<Translation2d> getCorals() { return corals; }

    public Pose3d getPose3d() {
        return new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
    }

    public Pose2d getPose2d() {
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    // Stops the server
    public void stop() {
        running = false;
        try {
            if (serverSocket != null) {
                serverSocket.close();
            }
        } catch (IOException e) {
            DriverStation.reportError("Failed to close localization server: " + e.getMessage(), false);
        }
    }
}