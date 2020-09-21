/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

import javax.swing.text.html.Option;
import java.util.List;
import java.util.Optional;

public class EMADynamicPortArraySymbol extends EMAPortArraySymbol {

    public static final EMADynamicPortArraySymbolKIND KIND = EMADynamicPortArraySymbolKIND.INSTANCE;

    protected boolean isDynamic;

    protected Optional<String> nameNonDynamicSizeDependsOn;
    protected int nonDynamicDimension;

    protected boolean dimensionInfinite = false;

    public EMADynamicPortArraySymbol(String name, String nameForNonDynamicDimension, String nameForDimension) {
        super(name, nameForDimension);
        this.setKind(KIND);
        this.nameNonDynamicSizeDependsOn = Optional.ofNullable(nameForNonDynamicDimension);
        isDynamic = false;
        nonDynamicDimension= 0;
    }

    public boolean isDynamic() {
        return isDynamic;
    }

    public void setDynamic(boolean dynamic) {
        isDynamic = dynamic;
    }

    public int getNonDynamicDimension() {
        return nonDynamicDimension;
    }

    public void setNonDynamicDimension(int nonDynamicDimension) {
        this.nonDynamicDimension = nonDynamicDimension;
    }

    public Optional<String> getNameNonDynamicSizeDependsOn(){
        return nameNonDynamicSizeDependsOn;
    }

    @Override
    public void recreatePortArray(ResolutionDeclarationSymbol resDeclSym, EmbeddedMontiArcSymbolTableCreator emastc, EMAComponentSymbolReference emaComponentSymbolReference) {
        Log.debug(emaComponentSymbolReference.toString(), "recreate");
        Log.debug(this.getNameSizeDependsOn().toString(), "String info:");
        if(this.getNameSizeDependsOn().isPresent() && this.getNameSizeDependsOn().get().equals(resDeclSym.getNameToResolve())){
            int size = -1;
            if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
                size = ((Double)((ASTUnitNumberResolution)resDeclSym.getASTResolution()).getNumber().get()).intValue();
            }
            if (size == 0) {
                size = this.getDimension();
                ((ASTUnitNumberResolution)resDeclSym.getASTResolution()).setNumber((double)size);
            }
            Log.debug(emaComponentSymbolReference.toString(), "FullName:");
            Log.debug(this.getDimension() + "", "old Dynamic Port Dimension:");
            Log.debug(size + "", "new Dynamic Port Dimension:");
            this.setDimension(size);
        }

        if(this.nameNonDynamicSizeDependsOn.isPresent() && this.nameNonDynamicSizeDependsOn.get().equals(resDeclSym.getNameToResolve())){
            int size = -1;
            if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
                size = ((Double)((ASTUnitNumberResolution)resDeclSym.getASTResolution()).getNumber().get()).intValue();
            }
            if (size == 0) {
                size = this.getNonDynamicDimension();
                ((ASTUnitNumberResolution)resDeclSym.getASTResolution()).setNumber((double)size);
            }
            Log.debug(emaComponentSymbolReference.toString(), "FullName:");
            Log.debug(this.getNonDynamicDimension() + "", "old static Port Dimension:");
            Log.debug(size + "", "new static Port Dimension:");
            this.setNonDynamicDimension(size);
        }

    }

    public boolean isDimensionInfinite() {
        return dimensionInfinite;
    }

    public void setDimensionInfinite(boolean dimensionInfinite) {
        this.dimensionInfinite = dimensionInfinite;
    }

    @Override
    public String getFullName() {
        return determineFullName();
    }

}
