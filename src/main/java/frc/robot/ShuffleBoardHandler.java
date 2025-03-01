package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ShuffleBoardHandler {

    private ShuffleboardTab tab;

    private GenericEntry alliance;

    private Optional<Alliance> ally = DriverStation.getAlliance();

    public ShuffleBoardHandler() {
        tab = Shuffleboard.getTab("Bruhdy");
        Shuffleboard.selectTab("Bruhdy");

        alliance = tab.add("Alliance Color", ally.isPresent()?(ally.get()==Alliance.Red?"Red":"Blue"):"N/A").withSize(2,1).withPosition(3, 3).getEntry();
    }

    public void update() {

        alliance.setString(ally.isPresent()?(ally.get()==Alliance.Red?"Red":"Blue"):"N/A");
    }
    
}
