/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTInterface;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTPort;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTInterfaceCoCo;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Checks that port names are unique (including implicit port names derived from ports without a
 * name).
 *
 */
public class UniquePorts implements EmbeddedMontiViewASTInterfaceCoCo {

  /**
   * @see EmbeddedMontiArcASTInterfaceCoCo#check(ASTInterface)
   */
  @Override
  public void check(ASTInterface node) {
    List<String> usedNames = new ArrayList<>();
    for (ASTPort port : node.getPorts()) {
      String name = "";
      if (port.getName().isPresent()) {
        name = port.getName().get();
      }
      else {
        // calc implicit name
        String implicitName = ArcTypePrinter.printType(port.getType().get());
        // TODO use symTab
        // ViewPortSymbol entry = ((ViewPortSymbol) port.getSymbol().get());
        // String implicitName = entry.getTypeReference().getReferencedSymbol().getName();
        name = StringTransformations.uncapitalize(implicitName);
      }
      if (usedNames.contains(name)) {
        Log.error(String.format("0xAC002 The name of port '%s' is ambiguos!", name), port.get_SourcePositionStart());
      }
      usedNames.add(name);

    }
  }

}
