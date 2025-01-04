package frc.robot.subsystems.Gyro;

import com.kauailabs.navx.frc.AHRS;

public class GyroIONavx implements GyroIO {
    private final AHRS navx = new AHRS();

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
