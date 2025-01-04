package frc.robot.subsystems.Gyro;

import com.studica.frc.AHRS;

public class GyroIONavx implements GyroIO {
    private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    public GyroIONavx() {
        navx.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yawPosition = navx.getRotation2d();
        inputs.yawVelocityRadPerSec = navx.getRate();
    }
}
