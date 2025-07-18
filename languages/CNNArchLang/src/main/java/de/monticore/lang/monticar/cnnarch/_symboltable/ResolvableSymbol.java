/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.Joiners;

import java.util.*;

public abstract class ResolvableSymbol extends CommonScopeSpanningSymbol {

    private Set<ParameterSymbol> unresolvableParameters = null;
    private ResolvableSymbol resolvedThis = null;

    protected ResolvableSymbol(String name, SymbolKind kind) {
        super(name, kind);
    }

    public ArchitectureSymbol getArchitecture(){
        Symbol sym = getEnclosingScope().getSpanningSymbol().get();
        if (sym instanceof ArchitectureSymbol){
            return (ArchitectureSymbol) sym;
        }
        else if (sym instanceof NetworkInstructionSymbol) {
            sym = sym.getEnclosingScope().getSpanningSymbol().get();
            return (ArchitectureSymbol) sym;
        }
        else {
            return ((ArchitectureElementSymbol) sym).getArchitecture();
        }
    }

    public Set<ParameterSymbol> getUnresolvableParameters() {
        if (unresolvableParameters == null){
            checkIfResolvable();
        }
        return unresolvableParameters;
    }

    protected void setUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters) {
        this.unresolvableParameters = unresolvableParameters;
    }

    public boolean isResolvable(){
        return getUnresolvableParameters().isEmpty();
    }

    public void checkIfResolvable(){
        checkIfResolvable(new HashSet<>());
    }

    protected void checkIfResolvable(Set<ParameterSymbol> occurringParameters){
        Set<ParameterSymbol> unresolvableParameters = new HashSet<>();
        computeUnresolvableParameters(unresolvableParameters, occurringParameters);
        setUnresolvableParameters(unresolvableParameters);
    }

    public Optional<ResolvableSymbol> getResolvedThis() {
        return Optional.ofNullable(resolvedThis);
    }

    protected void setResolvedThis(ResolvableSymbol resolvedThis) {
        if (resolvedThis != null && resolvedThis != this){
            resolvedThis.putInScope(getSpannedScope());
        }
        this.resolvedThis = resolvedThis;
    }

    public void resolveOrError() throws ArchResolveException{
        Set<ParameterSymbol> names = resolve();
        if (!isResolved()){
            throw new IllegalStateException("The following names could not be resolved: " + Joiners.COMMA.join(getUnresolvableParameters()));
        }
    }

    public boolean isResolved(){
        if (getResolvedThis().isPresent() && getResolvedThis().get() != this){
            return getResolvedThis().get().isResolved();
        }
        else {
            return getResolvedThis().isPresent();
        }
    }

    /**
     * resolves all expressions and underlying architecture elements and handles layer method calls and sequences.
     * Architecture parameters have to be set before calling resolve.
     * Resolves prepares the architecture elements such that the output type and shape of each element can be calculated and checked.
     * @return returns the set of all parameters which could not be resolved. Should be ignored.
     * @throws ArchResolveException thrown to interrupt the recursive resolve process to avoid follow-up Runtime Exceptions in tests after an error was logged.
     *                              Can be caught and ignored.
     */
    abstract public Set<ParameterSymbol> resolve() throws ArchResolveException;

    abstract protected void computeUnresolvableParameters(Set<ParameterSymbol> unresolvableParameters, Set<ParameterSymbol> allParameters);

    abstract protected void putInScope(Scope scope);

    abstract protected void resolveExpressions() throws ArchResolveException;

    /**
     * Creates a deep copy in the state before the architecture resolution.
     * @return returns a deep copy of this object in the pre-resolve version.
     */
    protected abstract ResolvableSymbol preResolveDeepCopy();
}
