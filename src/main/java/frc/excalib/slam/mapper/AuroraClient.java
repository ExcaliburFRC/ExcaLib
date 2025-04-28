package frc.excalib.slam.mapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

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

    public AuroraClient(int port) {
        serverThread = new Thread(() -> {
            try {
                serverSocket = new ServerSocket(port);
                DriverStation.reportWarning("Localization server started on port " + port, false);

                while (running) {
                    try (Socket clientSocket = serverSocket.accept();
                         DataInputStream in = new DataInputStream(clientSocket.getInputStream())) {

                        DriverStation.reportWarning("Localization client connected!", false);

                        while (running) {
                            try {
                                x = in.readFloat();
                                y = in.readFloat();
                                z = in.readFloat();
                                roll = in.readFloat();
                                pitch = in.readFloat();
                                yaw = in.readFloat();
                            } catch (IOException e) {
                                DriverStation.reportError("Error reading localization data: " + e.getMessage(), false);
                                break;
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
    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public float getZ() {
        return z;
    }

    public float getRoll() {
        return roll;
    }

    public float getPitch() {
        return pitch;
    }

    public float getYaw() {
        return yaw;
    }

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

