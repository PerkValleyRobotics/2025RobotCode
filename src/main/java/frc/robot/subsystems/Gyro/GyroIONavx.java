package frc.robot.subsystems.Gyro;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.subsystems.Drive.DriveConstants.*;

import java.util.Queue;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive.SparkOdometryThread;

public class GyroIONavx implements GyroIO {
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI, (byte) ODOMETRY_FREQUENCY);
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavx() {
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimeStampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navx::getAngle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navx.getAngle());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());
        
        inputs.odometryYawTimestamps = 
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = 
            yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(-value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
