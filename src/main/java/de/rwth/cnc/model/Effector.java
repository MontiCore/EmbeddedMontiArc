/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.model;

/**
 * represents an effector (very similar to Connector)
 */
public class Effector extends Connection {

  @Override
  public Effector clone() {
    Effector c = new Effector();
    c.setReceiver(receiver);
    c.setReceiverPort(receiverPort);
    c.setSender(sender);
    c.setSenderPort(senderPort);
    return c;
  }
}
