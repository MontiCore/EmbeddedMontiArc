package de.rwth.cnc.model;

public class Connection implements Cloneable {

  protected String sender = null;
  protected String receiver = null;
  protected String senderPort = null;
  protected String receiverPort = null;

  public boolean isComponentToComponent() {
    return (sender != null & receiver != null & (senderPort == null || senderPort.length() < 1) & (receiverPort == null || receiverPort.length() < 1));
  }

  public boolean isComponentToPort() {
    return (sender != null & receiver != null & (senderPort == null || senderPort.length() < 1) & receiverPort != null);
  }

  public boolean isPortToComponent() {
    return (sender != null & receiver != null & senderPort != null & (receiverPort == null || receiverPort.length() < 1));
  }

  public boolean isPortToPort() {
    return (sender != null & receiver != null & (senderPort != null && senderPort.length() > 0) & (receiverPort != null && receiverPort.length() > 0));
  }

  public String getSender() {
    return sender;
  }

  public void setSender(String sender) {
    assert sender != null : "sender must not be null!";
    assert !sender.contains(".") : "sender must not be the full name!";
    this.sender = sender;
  }

  public String getReceiver() {
    return receiver;
  }

  public void setReceiver(String receiver) {
    assert receiver != null : "receiver must not be null!";
    assert !receiver.contains(".") : "receiver must not be the full name!";
    this.receiver = receiver;
  }

  public String getSenderPort() {
    return senderPort;
  }

  public void setSenderPort(String senderPort) {
    assert senderPort == null || !senderPort.contains(".") : "senderPort must not be the full name!";
    this.senderPort = senderPort;
  }

  public String getReceiverPort() {
    return receiverPort;
  }

  public void setReceiverPort(String receiverPort) {
    assert receiverPort == null || !receiverPort.contains(".") : "receiverPort must not be the full name!";
    this.receiverPort = receiverPort;
  }

  public String getFullSender() {
    assert sender != null;
    if (senderPort == null)
      return sender;
    return sender + "." + senderPort;
  }

  public String getFullReceiver() {
    assert receiver != null;
    if (receiverPort == null)
      return receiver;
    return receiver + "." + receiverPort;
  }

  @Override
  public Connection clone() {
    Connection c = new Connection();
    c.setReceiver(receiver);
    c.setReceiverPort(receiverPort);
    c.setSender(sender);
    c.setSenderPort(senderPort);
    return c;
  }

  @Override
  public String toString() {
    return sender + "." + senderPort + " -> " + receiver + "." + receiverPort + ";";
  }
}
