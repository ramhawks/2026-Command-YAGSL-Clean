package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorRelay extends SubsystemBase {
    private final Relay relay;

    public AgitatorRelay(int relayChannel) {
        relay = new Relay(relayChannel, Relay.Direction.kForward);

        relay.set(Relay.Value.kOff);
    }

    public void on() {
        relay.set(Relay.Value.kOn);
    }

    public void off() {
        relay.set(Relay.Value.kOff);
    }

    public Command runWhileHelCommand() {
        return this.runEnd(this::on, this::off);
    }

    public Command runDuring(Command other) {
        return other.deadlineWith(runWhileHelCommand());
    }
}
