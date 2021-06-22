/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import org.apache.commons.math3.geometry.spherical.oned.Arc;

import java.util.*;

public class AdaNet extends PredefinedLayerDeclaration {
    private Optional<String> block_name;
    private Optional<LayerSymbol> block;
    private Optional<LayerSymbol> in;
    private Optional<LayerSymbol> out;
    private int defUnits;

    private AdaNet() {
        super(AllPredefinedLayers.AdaNet_Name);
        this.block_name = Optional.empty();
        this.defUnits = 100;

        this.block = Optional.empty(); // building block of the AdaNet Algorithm
        this.in = Optional.empty(); // input for the AdaNet Algorithm
        this.out = Optional.empty(); // output of the AdaNet Algorithm
    }


    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }

    public Optional<String> getBlock_name() {
        return block_name;
    }

    public void setBlock_name(Optional<String> newblock) {
        block_name = newblock;
    }

    public void setBlock(Optional<LayerSymbol> block) {
        this.block = block;
    }

    public Optional<LayerSymbol> getBlock() {
        return this.block;
    }

    public int getDefUnits() {
        return this.defUnits;
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        this.setBlock_name(layer.getStringValue(AllPredefinedLayers.Block)); // get Argument Value of block parameter
        ArchitectureElementSymbol workLayer = layer;
        String blockname = this.getBlock_name().get();  // name of the passed layer


        while (!workLayer.getPrevious().isEmpty() && !this.getBlock().isPresent()) {
            // search the passed element
            if (workLayer.getName().equals(blockname) && workLayer.isArtificial()) {
                //assure that the passed name leads to an artificial layer
                this.setBlock(Optional.of((LayerSymbol) workLayer));
            }
            workLayer = workLayer.getPrevious().get(0);
        }
        Optional<LayerDeclarationSymbol> lyr = Optional.empty();
        if (!this.getBlock().isPresent()) {

            for (LayerDeclarationSymbol name : AllPredefinedLayers.createList()) {
                if (name.getName().equals(blockname)) {
                    lyr = Optional.of((name));
                }
            }
            if (lyr.isPresent()) {
                List<ParameterSymbol> args = new ArrayList(Arrays.asList(
                        new ParameterSymbol.Builder()
                                .name(AllPredefinedLayers.UNITS_NAME)
                                .constraints(Constraints.INTEGER, Constraints.POSITIVE)
                                .defaultValue(this.getDefUnits())
                                .build(),
                        new ParameterSymbol.Builder()
                                .name(AllPredefinedLayers.NOBIAS_NAME)
                                .constraints(Constraints.BOOLEAN)
                                .defaultValue(false)
                                .build(),
                        new ParameterSymbol.Builder()
                                .name(AllPredefinedLayers.FLATTEN_PARAMETER_NAME)
                                .constraints(Constraints.BOOLEAN)
                                .defaultValue(true)
                                .build()
                ));
                List<ArgumentSymbol> Args = new ArrayList();
                for (ParameterSymbol arg : args) {
                    LayerDeclarationScope scp = this.getSpannedScope();
                    arg.putInScope(scp);
                    ArchSimpleExpressionSymbol exp = ArchSimpleExpressionSymbol.of(arg);
                    exp.setEnclosingScope(scp);

                    Args.add(new ArgumentSymbol.Builder()
                            .parameter(arg)
                            .value(exp)
                            .build());
                }

                this.setBlock(Optional.of(new LayerSymbol
                        .Builder()
                        .declaration(lyr.get())
                        .arguments(Args)
                        .build()));

            }
        }
        try {
            getBlock().get().resolve();
        } catch (Exception e) {
            System.out.print(e);
        }

        // TODO: AdaNet not supproted by curren backend error

        return this.getBlock().get().computeOutputTypes();
    }

    public static AdaNet create() {
        AdaNet declaration = new AdaNet();

        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol
                        .Builder()
                        .name(AllPredefinedLayers.Block)
                        .defaultValue(AllPredefinedLayers.FULLY_CONNECTED_NAME)
                        .build()
        ));

        declaration.setParameters(parameters);

        return declaration;
    }
}
