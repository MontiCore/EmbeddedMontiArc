/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast;

import de.monticore.lang.monticar.common2._ast.ASTParameter;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.types.types._ast.ASTReferenceType;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * TODO: Write me!
 *
 */
public class ASTComponent extends ASTComponentTOP {

    protected ASTComponent() {
        super();
    }

    protected ASTComponent(List<ASTComponentModifier> componentModifiers, String name, Optional<ASTTypeParameters2> genericTypeParameters, List<ASTParameter> parameters, Optional<ASTReferenceType> superComponent, ASTComponentBody body, boolean r__interface) {
        super(componentModifiers, name, genericTypeParameters, parameters, superComponent, body, r__interface);
    }

    // do not use symbol table, since symbol table must not be created
    public List<ASTPort> getPortsList() {
        List<ASTPort> ret = new ArrayList<>();
        for (ASTElement element : this.getBody().getElementList()) {
            if (element instanceof ASTInterface) {
                ret.addAll(((ASTInterface) element).getPortsList());
            }
        }
        return ret;
    }

    // do not use symbol table, since symbol table must not be created
    public List<ASTConnector> getConnectors() {
        return this.getBody().getElementList().stream().filter(a -> a instanceof ASTConnector).
                map(a -> (ASTConnector) a).collect(Collectors.toList());
    }

    // do not use symbol table, since symbol table must not be created
    public List<ASTSubComponent> getSubComponents() {
        return this.getBody().getElementList().stream().filter(a -> a instanceof ASTSubComponent).
                map(a -> (ASTSubComponent) a).collect(Collectors.toList());
    }

    // do not use symbol table, since symbol table must not be created
    public List<ASTComponent> getInnerComponents() {
        return this.getBody().getElementList().stream().filter(a -> a instanceof ASTComponent).
                map(a -> (ASTComponent) a).collect(Collectors.toList());
    }
}
