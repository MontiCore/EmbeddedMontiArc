/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._ast;

//import de.monticore.common.common._ast.ASTStereotype;
//import de.monticore.types.types._ast.ASTTypeArguments;

import de.monticore.lang.monticar.types2._ast.ASTTypeArguments;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * TODO: Write me!
 *
 */
public class ASTComponent extends ASTComponentTOP {
  /**
   * Constructor for de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent
   */
  public ASTComponent() {
    super();
  }

  protected ASTComponent(ASTIsAtomicTag IsAtomicTag, String name, ASTComponentHead head, String instanceName, ASTTypeArguments actualTypeArgument, ASTComponentBody body) {
    super(IsAtomicTag, name, head, instanceName, actualTypeArgument, body);
  }

  // do not use symbol table, since symbol table must not be created
  public List<ASTPort> getPorts() {
    List<ASTPort> ret = new ArrayList<>();
    for (ASTElement element : this.getBody().getElements()) {
      if (element instanceof ASTInterface) {
        ret.addAll(((ASTInterface) element).getPorts());
      }
    }
    return ret;
  }

  // do not use symbol table, since symbol table must not be created
  public List<ASTConnector> getConnectors() {
    return this.getBody().getElements().stream().filter(a -> a instanceof ASTConnector).
        map(a -> (ASTConnector) a).collect(Collectors.toList());
  }

  // do not use symbol table, since symbol table must not be created
  public List<ASTSubComponent> getSubComponents() {
    return this.getBody().getElements().stream().filter(a -> a instanceof ASTSubComponent).
        map(a -> (ASTSubComponent) a).collect(Collectors.toList());
  }

  // do not use symbol table, since symbol table must not be created
  public List<ASTComponent> getInnerComponents() {
    return this.getBody().getElements().stream().filter(a -> a instanceof ASTComponent).
        map(a -> (ASTComponent) a).collect(Collectors.toList());
  }
}
