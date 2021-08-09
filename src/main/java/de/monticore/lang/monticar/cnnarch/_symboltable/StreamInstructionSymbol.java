/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;

import java.util.Collection;
import java.util.Set;

public class StreamInstructionSymbol extends NetworkInstructionSymbol {

    public static final StreamInstructionKind KIND = new StreamInstructionKind();

    protected StreamInstructionSymbol() {
        super("", KIND);
    }

    @Override
    public boolean isStream() {
        return true;
    }

    public Set<ParameterSymbol> resolve() throws ArchResolveException {
        if (!isResolved()) {
            if (isResolvable()) {
                getBody().resolveOrError();

                setResolvedThis(this);
            }
        }

        return getUnresolvableParameters();
    }

    @Override
    public boolean containsAdaNet() {
        return super.containsAdaNet();
    }
    protected void resolveExpressions() throws ArchResolveException {}

    protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters) {}

    protected void putInScope(Scope scope) {
        Collection<Symbol> symbolsInScope = scope.getLocalSymbols().get(getName());
        if (symbolsInScope == null || !symbolsInScope.contains(this)){
            scope.getAsMutableScope().add(this);
        }
    }

    protected ResolvableSymbol preResolveDeepCopy() {
        StreamInstructionSymbol copy = new StreamInstructionSymbol();
        if (getAstNode().isPresent()){
            copy.setAstNode(getAstNode().get());
        }

        copy.setBody(getBody().preResolveDeepCopy());
        copy.getBody().putInScope(copy.getSpannedScope());

        return copy;
    }

}
