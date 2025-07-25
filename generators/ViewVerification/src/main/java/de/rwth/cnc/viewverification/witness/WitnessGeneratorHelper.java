/* (c) https://github.com/MontiCore/monticore */
package de.rwth.cnc.viewverification.witness;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import de.rwth.cnc.model.*;

public class WitnessGeneratorHelper {

  static final String witnessFileEnding = ".emv";

  public static void writeStringToFile(Path outputFile, String data) {
    try {
      Path tmp = outputFile.getParent();
      if (tmp != null) {
        Files.createDirectories(tmp);
      }
      if (Files.notExists(outputFile)) {
        Files.createFile(outputFile);
      }
      Files.write(outputFile, data.getBytes());
    }
    catch (Exception e) {
      e.printStackTrace();
    }
  }

  public static Path getWitnessFilePath(boolean isPositive, CnCView view, String topfolder, String name) {
    final String posOrNeg = isPositive ? "positive" : "negative";
    return Paths.get("target", "generated-witnesses", posOrNeg, view.getName(), topfolder, view.getPackageName().replace('.', '/'), name + witnessFileEnding);
  }

  public static Path getPositiveWitnessFilePath(CnCArchitecture model, CnCView view) {
    return getWitnessFilePath(true, view, "", "Witness_" + model.getName());
  }

  /**
   * returns the path containing parent and child component every component
   * except the child contains its child on the path
   *
   * @param parentName
   * @param childName
   * @return
   */
  public static List<Component> getArchPath(CnCArchitecture arch, String parentName, String childName) {
    Component parent = new Component();
    parent.setName(parentName);

    if (parentName.equals(childName)) {
      ArrayList<Component> path = new ArrayList<Component>();
      path.add(parent);
      return path;
    }
    for (String nextParentName : arch.getComponent(parentName).getContainedComponents()) {
      List<Component> path = getArchPath(arch, nextParentName, childName);
      if (path != null) {
        parent.addContainedComponent(nextParentName);
        path.add(0, parent);
        return path;
      }
    }
    return null;
  }

  /**
   * adds targets of the component and port specified adds all concrete
   * connectors leaving the adds also the target components with their ports and
   *
   * @param srcCmpAndPort
   * @param arch
   * @param witnessView
   */
  public static void addConnectorTargets(String srcCmpAndPort, CnCArchitecture arch, CnCView witnessView) {
    addConnectorTargets(srcCmpAndPort, srcCmpAndPort, arch, witnessView);
  }

  public static void addEffectorTargets(String srcCmpAndPort, CnCArchitecture arch, CnCView witnessView) {
    Set<String> addedCmpPorts = new HashSet<>();
    addEffectorTargets(srcCmpAndPort, srcCmpAndPort, arch, witnessView, addedCmpPorts);
  }

  /**
   * internal version to prevent cycles
   *
   * @param srcCmpAndPortStart
   * @param srcCmpAndPort
   * @param arch
   * @param witnessView
   */
  private static void addConnectorTargets(String srcCmpAndPortStart, String srcCmpAndPort, CnCArchitecture arch, CnCView witnessView) {
    Set<String> receivingPortsAndCmps = arch.getReceivingCmpsAndPorts(srcCmpAndPort);
    for (String cmpAndPort : receivingPortsAndCmps) {
      String rcvCmpName = cmpAndPort.split("\\.")[0];
      String rcvPortName = cmpAndPort.split("\\.")[1];
      witnessView.addComponentWithIntermediateLayers(arch, rcvCmpName);

      copyPortToView(arch, witnessView, rcvCmpName, rcvPortName);

      Connection connection = new Connection();
      connection.setReceiver(cmpAndPort.split("\\.")[0]);
      connection.setReceiverPort(cmpAndPort.split("\\.")[1]);
      connection.setSender(srcCmpAndPort.split("\\.")[0]);
      connection.setSenderPort(srcCmpAndPort.split("\\.")[1]);
      witnessView.addConnection(connection);

      if (!srcCmpAndPortStart.equals(cmpAndPort)) {
        addConnectorTargets(srcCmpAndPortStart, cmpAndPort, arch, witnessView);
      }
    }
  }

  private static void addEffectorTargets(String srcCmpAndPortStart, String srcCmpAndPort, CnCArchitecture arch, CnCView witnessView, Set<String> addedCmpPorts) {
    Set<String> receivingPortsAndCmps = arch.getEffectedCmpsAndPorts(srcCmpAndPort);

    for (String cmpAndPort : receivingPortsAndCmps) {
      if (addedCmpPorts.contains(cmpAndPort))
        continue; //to avoid loops

      addedCmpPorts.add(cmpAndPort);

      String rcvCmpName = cmpAndPort.split("\\.")[0];
      String rcvPortName = cmpAndPort.split("\\.")[1];
      witnessView.addComponentWithIntermediateLayers(arch, rcvCmpName);

      copyPortToView(arch, witnessView, rcvCmpName, rcvPortName);

      String receiver = cmpAndPort.split("\\.")[0];
      String receiverPort = cmpAndPort.split("\\.")[1];
      String sender = srcCmpAndPort.split("\\.")[0];
      String senderPort = srcCmpAndPort.split("\\.")[1];

      if (receiver.equals(sender)) {
        Effector effector = new Effector();
        effector.setReceiver(receiver);
        effector.setReceiverPort(receiverPort);
        effector.setSender(sender);
        effector.setSenderPort(senderPort);
        witnessView.addEffector(effector);
      }
      else {
        Connection connection = new Connection();
        connection.setReceiver(receiver);
        connection.setReceiverPort(receiverPort);
        connection.setSender(sender);
        connection.setSenderPort(senderPort);
        witnessView.addConnection(connection);
      }

      if (!srcCmpAndPortStart.equals(cmpAndPort)) {
        addEffectorTargets(srcCmpAndPortStart, cmpAndPort, arch, witnessView, addedCmpPorts);
      }
    }

  }

  /**
   * checks existence first
   *
   * @param arch
   * @param witnessView
   * @param rcvCmpName
   * @param rcvPortName
   */
  public static void copyPortToView(CnCArchitecture arch, CnCView witnessView, String rcvCmpName, String rcvPortName) {

    if (witnessView.getComponent(rcvCmpName).getPort(rcvPortName) == null) {
      Port p = arch.getComponent(rcvCmpName).getPort(rcvPortName);
      witnessView.getComponent(rcvCmpName).addPort(p.clone());
    }
  }

  public static void copyPorts(String cmpName, CnCArchitecture arch, CnCView view) {
    Component viewCmp = view.getComponent(cmpName);
    Component archCmp = arch.getComponent(cmpName);

    for (Port p : archCmp.getPorts()) {
      viewCmp.addPort(p.clone());
    }
  }

}
