/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import com.google.common.collect.ImmutableList;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.CommonScopeSpanningSymbol;
import de.monticore.symboltable.MutableScope;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkArgument;

/**
 */
public abstract class CommonMCTypeSymbol<T extends MCTypeSymbol, V extends MCTypeReference<T>>
        extends CommonScopeSpanningSymbol implements MCTypeSymbol {

    private final List<V> interfaces = new ArrayList<>();
    private V superClass;
    private boolean isFormalTypeParameter = false;

    protected CommonMCTypeSymbol(String name) {
        this(name, MCTypeSymbol.KIND);
    }

    protected CommonMCTypeSymbol(String name, MCTypeSymbolKind typeKind) {
        super(name, typeKind);
    }

    @Override
    protected MutableScope createSpannedScope() {
        return new CommonMCTypeScope(Optional.empty());
    }

    public void addFormalTypeParameter(T formalTypeParameter) {
        checkArgument(formalTypeParameter.isFormalTypeParameter());
        getMutableSpannedScope().add(formalTypeParameter);
    }

    @Override
    public List<T> getFormalTypeParameters() {
        final Collection<T> resolvedTypes = getSpannedScope().resolveLocally(T.KIND);
        return resolvedTypes.stream().filter(T::isFormalTypeParameter).collect(Collectors.toList());
    }

    @Override
    public Optional<V> getSuperClass() {
        return Optional.ofNullable(superClass);
    }

    public void setSuperClass(V superClass) {
        this.superClass = superClass;
    }

    @Override
    public List<V> getInterfaces() {
        return ImmutableList.copyOf(interfaces);
    }

    public void addInterface(V superInterface) {
        this.interfaces.add(Log.errorIfNull(superInterface));
    }

    @Override
    public List<V> getSuperTypes() {
        final List<V> superTypes = new ArrayList<>();
        if (getSuperClass().isPresent()) {
            superTypes.add(getSuperClass().get());
        }
        superTypes.addAll(getInterfaces());
        return superTypes;
    }

    @Override
    public boolean isFormalTypeParameter() {
        return isFormalTypeParameter;
    }

    public void setFormalTypeParameter(boolean formalTypeParameter) {
        this.isFormalTypeParameter = formalTypeParameter;
    }
}
