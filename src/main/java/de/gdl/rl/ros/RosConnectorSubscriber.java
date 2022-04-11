package de.gdl.rl.ros;
public interface RosConnectorSubscriber {
    public void receivedData(String topic, boolean value);
    public void receivedData(String topic, int value);
}