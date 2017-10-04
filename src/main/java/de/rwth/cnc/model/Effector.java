package de.rwth.cnc.model;

/**
 * represents an effector (very similar to Connector)
 */
public class Effector extends Connection {

  @Override
  public Connection clone() {
    Connection c = new Effector();
    c.setReceiver(receiver);
    c.setReceiverPort(receiverPort);
    c.setSender(sender);
    c.setSenderPort(senderPort);
    return c;
  }
}
