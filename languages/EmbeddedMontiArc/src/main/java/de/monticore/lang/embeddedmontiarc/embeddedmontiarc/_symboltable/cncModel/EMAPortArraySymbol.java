/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.logging.Log;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Symboltable entry for port arrays
 */
public class EMAPortArraySymbol extends EMAPortSymbol {
    public static final PortArraySymbolKind KIND = PortArraySymbolKind.INSTANCE;

    protected Optional<String> nameSizeDependsOn;

    public EMAPortArraySymbol(String name, String nameSizeDependsOn) {
        super(name, KIND);
        this.nameSizeDependsOn = Optional.ofNullable(nameSizeDependsOn);
        Log.debug(getFullName(), "EMAPortArraySymbol ");
        Log.debug(this.nameSizeDependsOn.orElse(null), "set NameSizeDependsOn to:");
    }

    private int dimension = 1;

    public Optional<String> getNameSizeDependsOn() {
        return nameSizeDependsOn;
    }

    public int getDimension() {
        return dimension;
    }

    public void setDimension(int dimension) {
        this.dimension = dimension;
    }

    public List<? extends EMAPortSymbol> getConcretePortSymbols() {
        //TODO fix wrong port return
        return getEnclosingScope().<EMAPortSymbol>resolveLocally(EMAPortSymbol.KIND)
                .stream().filter(s -> s.getName().startsWith(this.getName()/*+"["*/))
                .collect(Collectors.toList());
    }

    /**
     * starts with 1
     *
     * @param index
     * @return
     */
    public Optional<EMAPortSymbol> getPortSymbolWithIndex(int index) {
        for (EMAPortSymbol emaPortSymbol : getConcretePortSymbols()) {
            if (emaPortSymbol.getName().contains("[" + index + "]")) {
                return Optional.of(emaPortSymbol);
            }
        }
        return Optional.ofNullable(null);
    }

    public void recreatePortArray(ResolutionDeclarationSymbol resDeclSym, EmbeddedMontiArcSymbolTableCreator emastc, EMAComponentSymbolReference emaComponentSymbolReference) {
        Log.debug(emaComponentSymbolReference.toString(), "recreate");
        Log.debug(getNameSizeDependsOn().toString(), "String info:");
        if (getNameSizeDependsOn().isPresent() && getNameSizeDependsOn().get().equals(resDeclSym.getNameToResolve())) {
            int size = -1;
            if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
                size = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getNumber().get().intValue();
            }
            List<? extends EMAPortSymbol> portSymbols = getConcretePortSymbols();

            EMAPortSymbol firstPort = getPortSymbolWithIndex(1).get();

            int oldSize = portSymbols.size();
            if (size == 0) {
                size = oldSize;
                ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).setNumber(Double.valueOf(oldSize));
            }
            Log.debug(emaComponentSymbolReference.toString(), "FullName:");
            Log.debug(oldSize + "", "old Port Size:");
            Log.debug(size + "", "new Port Size:");

            for (int i = 0; i <= size; ++i) {
                if (oldSize < i) {
                    //Log.debug();
                    createPortSymbolForArrayIndex(emaComponentSymbolReference, (ASTPort) firstPort.getAstNode().get(), this.getName() + "[" + i + "]", firstPort.getTypeReference());
                }
            }
            //just add missing ports here and fix actual size after emacomponentinstance creation
            /*for (int i = size + 1; i <= oldSize; ++i) {
                if (getPortSymbolWithIndex(i).isPresent())
                    getEnclosingScope().getAsMutableScope().remove(getPortSymbolWithIndex(i).get());
            }
            */
        } else {
            Log.debug("Is not Present", "NameSizeDependsOn:");
        }
    }

    private void createPortSymbolForArrayIndex(EMAComponentSymbolReference emaComponentSymbolReference, ASTPort node, String name, MCTypeReference<? extends MCTypeSymbol> typeRef) {
        EMAPortSymbol ps = new EMAPortSymbol(name);
        ps.setNameDependsOn(nameSizeDependsOn);
        ps.setTypeReference(typeRef);
        ps.setDirection(node.isIncoming());

        getEnclosingScope().getAsMutableScope().add(ps);

        Log.debug(name + " " + emaComponentSymbolReference.getAllIncomingPorts().size(), "Added EMAPortSymbol From PortArray:");
    }


    public static class PortArraySymbolKind implements SymbolKind {

        public static final PortArraySymbolKind INSTANCE = new PortArraySymbolKind();

        protected PortArraySymbolKind() {

        }
    }
}
