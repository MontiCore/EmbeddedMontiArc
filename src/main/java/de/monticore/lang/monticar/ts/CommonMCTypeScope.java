/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.ts;

import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.*;
import de.monticore.symboltable.modifiers.AccessModifier;
import de.monticore.symboltable.resolving.ResolvingInfo;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.Optional;

import static com.google.common.base.Preconditions.checkArgument;
import static de.monticore.symboltable.modifiers.AccessModifier.ALL_INCLUSION;
import static de.monticore.symboltable.modifiers.BasicAccessModifier.*;

/**
 */
public class CommonMCTypeScope extends CommonScope {

    public CommonMCTypeScope(Optional<MutableScope> enclosingScope) {
        super(enclosingScope, true);
    }

    @Override
    @SuppressWarnings("unchecked")
    public Optional<? extends MCTypeSymbol> getSpanningSymbol() {
        return (Optional<? extends MCTypeSymbol>) super.getSpanningSymbol();
    }

    @Override
    public void setSpanningSymbol(ScopeSpanningSymbol symbol) {
        checkArgument(symbol instanceof MCTypeSymbol);
        super.setSpanningSymbol(symbol);
    }

    @Override
    public <T extends Symbol> Optional<T> resolve(String symbolName, SymbolKind kind) {
        return this.resolve(symbolName, kind, ALL_INCLUSION);
    }

    @Override
    public <T extends Symbol> Optional<T> resolve(String name, SymbolKind kind, AccessModifier modifier) {
        Optional<T> resolvedSymbol = this.resolveImported(name, kind, modifier);

        if (!resolvedSymbol.isPresent()) {
            resolvedSymbol = resolveInSuperTypes(name, kind, modifier);
        }

        if (!resolvedSymbol.isPresent()) {
            // continue with enclosing scope
            resolvedSymbol = super.resolve(name, kind, modifier);
        }

        return resolvedSymbol;
    }

    protected <T extends Symbol> Optional<T> resolveInSuperTypes(String name, SymbolKind kind, AccessModifier modifier) {
        Optional<T> resolvedSymbol = Optional.empty();

        final MCTypeSymbol spanningSymbol = getSpanningSymbol().get();

        // resolve in super class
        if (spanningSymbol.getSuperClass().isPresent()) {
            final MCTypeSymbol superClass = spanningSymbol.getSuperClass().get().getReferencedSymbol();
            resolvedSymbol = resolveInSuperType(name, kind, modifier, superClass);
        }

        // resolve in interfaces
        if (!resolvedSymbol.isPresent()) {
            for (MCTypeReference<? extends MCTypeSymbol> interfaceRef : spanningSymbol.getInterfaces()) {
                final MCTypeSymbol interfaze = interfaceRef.getReferencedSymbol();
                resolvedSymbol = resolveInSuperType(name, kind, modifier, interfaze);

                // Stop as soon as symbol is found in an interface. Note that the other option is to
                // search in all interfaces and throw an ambiguous exception if more than one symbol is
                // found. => TODO discuss it!
                if (resolvedSymbol.isPresent()) {
                    break;
                }
            }
        }

        return resolvedSymbol;
    }

    private <T extends Symbol> Optional<T> resolveInSuperType(String name, SymbolKind kind,
                                                              final AccessModifier modifier, MCTypeSymbol superType) {

        Log.trace("Continue in scope of super class " + superType.getName(), CommonMCTypeScope.class
                .getSimpleName());
        // Private symbols cannot be resolved from the super class. So, the modifier must at
        // least be protected when searching in the super class scope
        AccessModifier modifierForSuperClass = getModifierForSuperClass(modifier, superType);

        return superType.getSpannedScope().resolveImported(name, kind, modifierForSuperClass);
    }

    private AccessModifier getModifierForSuperClass(AccessModifier modifier, MCTypeSymbol superType) {
        if (modifier.equals(ALL_INCLUSION) || modifier.equals(PRIVATE) || modifier.equals(PACKAGE_LOCAL)) {
            if (getSpanningSymbol().get().getPackageName().equals(superType.getPackageName())) {
                return PACKAGE_LOCAL;
            } else {
                return PROTECTED;
            }
        }
        return modifier;
    }

    @Override
    public <T extends Symbol> Optional<T> resolveImported(String name, SymbolKind kind, AccessModifier modifier) {
        final Collection<T> resolvedSymbols = resolveManyLocally(new ResolvingInfo(getResolvingFilters()), name, kind, modifier, x -> true);

        if (resolvedSymbols.isEmpty()) {
            return resolveInSuperTypes(name, kind, modifier);
        }

        return getResolvedOrThrowException(resolvedSymbols);
    }
}
