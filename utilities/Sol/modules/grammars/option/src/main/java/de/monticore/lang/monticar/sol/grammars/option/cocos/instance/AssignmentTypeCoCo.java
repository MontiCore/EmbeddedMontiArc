/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos.instance;

import de.monticore.lang.monticar.sol.grammars.option._ast.ASTPropAssignment;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionASTPropAssignmentCoCo;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.PropDeclarationSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralListAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option.cocos.CommonOptionCoCo;

/**
 * This context condition checks whether a given assignment has the correct type.
 */
public class AssignmentTypeCoCo extends CommonOptionCoCo implements OptionASTPropAssignmentCoCo {
    public static final String UNKNOWN_TYPE = "unknown";

    public AssignmentTypeCoCo() {
        super("OPT0000", "'%s' is not a valid value for '%s' of type '%s'.");
    }

    @Override
    public void registerTo(OptionCoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTPropAssignment node) {
        node.getPropAssignmentSymbolOpt().ifPresent(this::check);
    }

    protected void check(PropAssignmentSymbol assignment) {
        assignment.getDeclarationSymbol().ifPresent(declaration -> this.check(assignment, declaration));
    }

    protected void check(PropAssignmentSymbol assignment, PropDeclarationSymbol declaration) {
        String declarationType = declaration.getType().orElse(UNKNOWN_TYPE);

        assignment.asLiteralAssignment().ifPresent(a -> this.check(a, declarationType));
        assignment.asLiteralListAssignment().ifPresent(a -> this.check(a, declarationType));
    }

    protected void check(LiteralAssignmentSymbol assignment, String declarationType) {
        String assignmentType = assignment.getType().orElse(UNKNOWN_TYPE);

        if (!assignmentType.equals(declarationType)) this.error(assignment, assignment.getValue(), assignment.getName(), declarationType);
    }

    protected void check(LiteralListAssignmentSymbol assignment, String declarationType) {
        String assignmentType = assignment.getType().orElse(UNKNOWN_TYPE);

        if (!assignmentType.equals(declarationType)) this.error(assignment, assignment.getValues(), assignment.getName(), declarationType);
    }
}
