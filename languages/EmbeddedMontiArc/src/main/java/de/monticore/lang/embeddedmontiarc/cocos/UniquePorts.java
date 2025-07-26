/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTInterface;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTInterfaceCoCo;
import de.monticore.lang.embeddedmontiarc.helper.ArcTypePrinter;
import de.se_rwth.commons.StringTransformations;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Checks that port names are unique (including implicit port names derived from ports without a
 * name).
 *
 */
public class UniquePorts implements EmbeddedMontiArcASTInterfaceCoCo {

  /**
   * @see EmbeddedMontiArcASTInterfaceCoCo#check(ASTInterface)
   */
  @Override
  public void check(ASTInterface node) {
    List<String> usedNames = new ArrayList<>();
    for (ASTPort port : node.getPortsList()) {
      String name = "";
      if (port.getNameOpt().isPresent()) {
        name = port.getNameOpt().get();
      }
      else {
        // calc implicit name
        String implicitName = ArcTypePrinter.printType(port.getType());
        // TODO use symTab
        // EMAPortSymbol entry = ((EMAPortSymbol) port.getSymbolOpt().get());
        // String implicitName = entry.getTypeReference().getReferencedSymbol().getName();
        name = StringTransformations.uncapitalize(implicitName);
      }
      if (usedNames.contains(name)) {
        Log.error(String.format("0xAC002 The name of port '%s' is ambiguos!", name),
            port.get_SourcePositionStart());
      }
      usedNames.add(name);

    }
  }

}
