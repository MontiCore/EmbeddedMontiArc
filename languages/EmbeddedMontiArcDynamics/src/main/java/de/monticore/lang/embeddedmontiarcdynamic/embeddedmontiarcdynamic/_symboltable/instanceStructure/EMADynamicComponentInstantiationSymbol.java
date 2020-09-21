/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcSymbolTableCreator;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbolReference;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberResolution;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class EMADynamicComponentInstantiationSymbol extends EMAComponentInstantiationSymbol {

    public static final EMADynamicComponentInstantiationKind KIND = EMADynamicComponentInstantiationKind.INSTANCE;

    protected boolean dynamic = false;
    protected boolean array = false;
    protected int dimension = 0;

    protected int nonDynamicDimension = 0;

    protected Optional<String> dimensionDependsOn;
    protected Optional<String> nonDynamicDimensionDependsOn;

    protected boolean dimensionInfinite = false;

    public EMADynamicComponentInstantiationSymbol(String name, EMADynamicComponentSymbolReference componentType){
        super(name, componentType);
        this.setKind(EMADynamicComponentInstantiationSymbol.KIND);

        dimensionDependsOn = Optional.empty();
        nonDynamicDimensionDependsOn = Optional.empty();

        dimension = 0;
        nonDynamicDimension = 0;
    }

    public EMADynamicComponentInstantiationSymbol(String name, EMADynamicComponentSymbolReference componentType,
                                                  String dimensionDependsOnName){
        super(name, componentType);
        this.setKind(EMADynamicComponentInstantiationSymbol.KIND);

        dimensionDependsOn = Optional.ofNullable(dimensionDependsOnName);
        nonDynamicDimensionDependsOn = Optional.empty();

        dimension = 0;
        nonDynamicDimension = 0;
        dimensionInfinite = false;
    }

    public EMADynamicComponentInstantiationSymbol(String name, EMADynamicComponentSymbolReference componentType,
                                                  String nonDynamicDimensionDependsOnName, String dimensionDependsOnName){
        super(name, componentType);
        this.setKind(EMADynamicComponentInstantiationSymbol.KIND);

        dimensionDependsOn = Optional.ofNullable(dimensionDependsOnName);
        nonDynamicDimensionDependsOn = Optional.ofNullable(nonDynamicDimensionDependsOnName);

        dimension = 0;
        nonDynamicDimension = 0;
        dimensionInfinite = false;
    }

    public boolean isDynamic() {
        return dynamic;
    }

    public void setDynamic(boolean dynamic){
        this.dynamic = dynamic;
    }

    public boolean isArray() {
        return array;
    }

    public void setArray(boolean array) {
        this.array = array;
    }

    public int getDimension(){
        return dimension;
    }

    public void setDimension(int numberOfDynamicInstances) {
        this.dimension = numberOfDynamicInstances;
    }

    public int getNonDynamicDimension() {
        return nonDynamicDimension;
    }

    public void setNonDynamicDimension(int nonDynamicDimension) {
        this.nonDynamicDimension = nonDynamicDimension;
    }

    public Optional<String> getDimensionDependsOn() {
        return dimensionDependsOn;
    }

    public void setDimensionDependsOn(Optional<String> dimensionDependsOn) {
        this.dimensionDependsOn = dimensionDependsOn;
    }

    public Optional<String> getNonDynamicDimensionDependsOn() {
        return nonDynamicDimensionDependsOn;
    }

    public EMADynamicComponentSymbolReference getDynamicComponentType() {
        return (EMADynamicComponentSymbolReference)this.getComponentType();
    }

    @Override
    public String toString() {
        return super.toString();
    }

    public void fixResolutions(ResolutionDeclarationSymbol resDeclSym, EmbeddedMontiArcSymbolTableCreator emastc, EMAComponentSymbolReference emaComponentSymbolReference) {

        if(this.isArray()){
            if(this.getDimensionDependsOn().isPresent() && this.getDimensionDependsOn().get().equals(resDeclSym.getNameToResolve())){
                int size = -1;
                if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
                    size = ((Double)((ASTUnitNumberResolution)resDeclSym.getASTResolution()).getNumber().get()).intValue();
                }
                if (size == 0) {
                    size = this.getDimension();
                    ((ASTUnitNumberResolution)resDeclSym.getASTResolution()).setNumber((double)size);
                }
                Log.debug(emaComponentSymbolReference.toString(), "FullName:");
                Log.debug(this.getDimension() + "", "old (dynamic) subcomponent instance array Dimension:");
                Log.debug(size + "", "new (dynamic) subcomponent instance array Dimension:");
                this.setDimension(size);
            }

            if(this.getNonDynamicDimensionDependsOn().isPresent() && this.getNonDynamicDimensionDependsOn().get().equals(resDeclSym.getNameToResolve())){
                int size = -1;
                if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
                    size = ((Double)((ASTUnitNumberResolution)resDeclSym.getASTResolution()).getNumber().get()).intValue();
                }
                if (size == 0) {
                    size = this.getNonDynamicDimension();
                    ((ASTUnitNumberResolution)resDeclSym.getASTResolution()).setNumber((double)size);
                }
                Log.debug(emaComponentSymbolReference.toString(), "FullName:");
                Log.debug(this.getDimension() + "", "old dynamic subcomponent instance array non dynamic Dimension:");
                Log.debug(size + "", "new dynamic subcomponent instance array non dynamic Dimension:");
                this.setNonDynamicDimension(size);
            }


        }

    }

    public  String getNameWithoutArrayBracketPart() {
        String nameWithOutArrayBracketPart = this.getName();
        if (nameWithOutArrayBracketPart.endsWith("]")) {
            char lastChar;
            do {
                lastChar = nameWithOutArrayBracketPart.charAt(nameWithOutArrayBracketPart.length() - 1);
                nameWithOutArrayBracketPart = nameWithOutArrayBracketPart.substring(0,
                        nameWithOutArrayBracketPart.length() - 1);
            } while (lastChar != '[');
        }
        return nameWithOutArrayBracketPart;
    }

    public boolean isDimensionInfinite() {
        return dimensionInfinite;
    }

    public void setDimensionInfinite(boolean dimensionInfinite) {
        this.dimensionInfinite = dimensionInfinite;
    }
}
