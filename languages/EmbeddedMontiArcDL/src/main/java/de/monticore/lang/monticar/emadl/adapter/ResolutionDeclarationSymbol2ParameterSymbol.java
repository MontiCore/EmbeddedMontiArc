/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.adapter;

import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterType;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.resolving.SymbolAdapter;
import de.se_rwth.commons.SourcePosition;
import org.jscience.mathematics.number.Rational;

import java.util.Optional;

public class ResolutionDeclarationSymbol2ParameterSymbol extends ParameterSymbol
        implements SymbolAdapter<ResolutionDeclarationSymbol> {

    private final ResolutionDeclarationSymbol adaptee;

    public ResolutionDeclarationSymbol2ParameterSymbol(ResolutionDeclarationSymbol ps, ASTUnitNumberResolution unitNumberResolution) {
        super(ps.getName());
        setType(ParameterType.ARCHITECTURE_PARAMETER);
        Double doubleValue = unitNumberResolution.getNumberWithUnit().getNumber().get();
        setExpression(ArchSimpleExpressionSymbol.of((doubleValue % 1)!= 0
                ? doubleValue.intValue()
                : doubleValue));
        this.adaptee = ps;
    }

    @Override
    public ResolutionDeclarationSymbol getAdaptee() {
        return adaptee;
    }

    @Override
    public Optional<ASTNode> getAstNode() {
        return getAdaptee().getAstNode();
    }

    @Override
    public SourcePosition getSourcePosition() {
        return getAdaptee().getSourcePosition();
    }

}
