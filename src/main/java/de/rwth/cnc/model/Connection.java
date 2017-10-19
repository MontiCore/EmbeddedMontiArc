/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.cnc.model;

public class Connection implements Cloneable {

  protected String sender = null;
  protected String receiver = null;
  protected String senderPort = null;
  protected String receiverPort = null;

  public boolean isComponentToComponent() {
    return (sender != null & receiver != null & (senderPort == null || senderPort.length() < 1 || senderPort.startsWith("$")) & (receiverPort == null || receiverPort.length() < 1 || receiverPort.startsWith("$")));
  }

  public boolean isComponentToPort() {
    return (sender != null & receiver != null & (senderPort == null || senderPort.length() < 1 || senderPort.startsWith("$")) & receiverPort != null);
  }

  public boolean isPortToComponent() {
    return (sender != null & receiver != null & senderPort != null & (receiverPort == null || receiverPort.length() < 1)|| receiverPort.startsWith("$"));
  }

  public boolean isPortToPort() {
    return (sender != null & receiver != null & (senderPort != null && senderPort.length() > 0 && !senderPort.startsWith("$")) & (receiverPort != null && receiverPort.length() > 0) && !receiverPort.startsWith("$"));
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
    if (sender == "")
      return senderPort;
    return sender + "." + senderPort;
  }

  public String getFullReceiver() {
    assert receiver != null;
    if (receiverPort == null)
      return receiver;
    if (receiver == "")
      return receiverPort;
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
