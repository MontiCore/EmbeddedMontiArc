/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package de.monticore.lang.monticar.cnnarch.predefined;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.symboltable.Symbol;
import org.apache.commons.math3.geometry.spherical.oned.Arc;
import org.checkerframework.checker.units.qual.A;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;
import de.se_rwth.commons.logging.Log;
import java.security.InvalidParameterException;
import java.util.*;

import static de.monticore.lang.monticar.cnnarch.helper.ErrorCodes.MISSING_VAR_VALUE;

public class AdaNet extends PredefinedLayerDeclaration {
    private Optional<String> block_name;
    private Optional<String> in_name;
    private Optional<String> out_name;
    private Optional<ArchitectureElementSymbol> block;
    private Optional<ArchitectureElementSymbol> in;
    private Optional<ArchitectureElementSymbol> out;


    private AdaNet() {
        super(AllPredefinedLayers.AdaNet_Name);
        this.block_name = Optional.empty();
        this.in_name = Optional.empty();
        this.out_name = Optional.empty();

        this.block = Optional.empty(); // building block of the AdaNet Algorithm
        this.in = Optional.empty(); // input for the AdaNet Algorithm
        this.out = Optional.empty(); // output of the AdaNet Algorithm
    }


    @Override
    public void checkInput(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        errorIfInputSizeIsNotOne(inputTypes, layer);
    }


    public void setBlock_name(Optional<String> newblock) {
        block_name = newblock;
    }

    public void setIn_name(Optional<String> name) {
        this.in_name = name;
    }

    public void setOut_name(Optional<String> name) {
        this.out_name = name;
    }

    public Optional<String> getBlock_name() {
        return block_name;
    }

    public Optional<String> getIn_name() {
        return this.in_name;
    }

    public Optional<String> getOut_name() {
        return this.out_name;
    }

    public void setIn(Optional<ArchitectureElementSymbol> in) {
        this.in = in;
    }

    public void setOut(Optional<ArchitectureElementSymbol> out) {
        this.out = out;
    }

    public Optional<ArchitectureElementSymbol> getOut() {
        return out;
    }

    public Optional<ArchitectureElementSymbol> getIn() {
        return in;
    }

    public void setBuildBlock(Optional<ArchitectureElementSymbol> block) {
        this.block = block;
    }

    public Optional<ArchitectureElementSymbol> getBuildBlock() {
        return this.block;
    }


    public void setBlock(String name, Optional<ArchitectureElementSymbol> block) {

        switch (name) {
            case AllPredefinedLayers.Block:
                setBuildBlock(block);
                break;
            case AllPredefinedLayers.In:
                setIn(block);
                break;
            case AllPredefinedLayers.Out:
                setOut(block);
                break;
            default:
                throw new InvalidParameterException(String.format("Parameter name should be '%s', '%s' or '%s'. got %s",
                        AllPredefinedLayers.Block,
                        AllPredefinedLayers.In,
                        AllPredefinedLayers.Out,
                        name));
        }
    }

    public Optional<ArchitectureElementSymbol> getBlock(String name) {
        // returns the inBlock,outBlock or buildingBlock depending on target
        switch (name) {
            case AllPredefinedLayers.In:
                return getIn();
            case AllPredefinedLayers.Out:
                return getOut();
            case AllPredefinedLayers.Block:
                return getBuildBlock();
            default:
                throw new InvalidParameterException(String.format("Parameter name should be '%s', '%s' or '%s'. got %s",
                        AllPredefinedLayers.Block,
                        AllPredefinedLayers.In,
                        AllPredefinedLayers.Out,
                        name));
        }

    }

    public Optional<String> getName(String target) {
        switch (target) {
            case AllPredefinedLayers.Block:
                return getBlock_name();
            case AllPredefinedLayers.In:
                return getIn_name();
            case AllPredefinedLayers.Out:
                return getOut_name();
            default:
                return Optional.empty();
        }
    }

    private void connectBlock(ArchitectureElementSymbol block, LayerSymbol layer, ArchitectureElementSymbol input, ArchitectureElementSymbol output) {
        block.setInputElement(input);
        block.setOutputElement(output);
        if (block.isArtificial()) {
            connectBlock(block, layer);
        }
    }

    private void connectBlock(ArchitectureElementSymbol block, LayerSymbol layer) {
        ArchitectureElementSymbol input = layer.getInputElement().get();
        ArchitectureElementSymbol output = layer.getOutputElement().get();

        if (block.isArtificial()) {
            for (List<ArchitectureElementSymbol> subNetwork : ((LayerSymbol) block).getDeclaration().getBody().getEpisodicSubNetworks()) {
                ArchitectureElementSymbol previous = null;
                for (ArchitectureElementSymbol lay : subNetwork) {
                    if (previous == null) { // check if lay is the first element in the list
                        lay.setInputElement(input);
                    } else {
                        lay.setInputElement(previous);
                        previous.setOutputElement(lay);
                    }
                    if (lay.equals(subNetwork.get(subNetwork.size() - 1))) { // check if lay is the last element
                        lay.setOutputElement(output);
                    }
                    if (lay.isArtificial()) {
                        connectBlock(lay, (LayerSymbol) block);
                    }
                }
            }
        } else {
            block.setInputElement(input);
            block.setOutputElement(output);
        }

    }

    private void connectAdaNet(LayerSymbol Ada) {
        Optional<ArchitectureElementSymbol> input = Optional.empty();
        Optional<ArchitectureElementSymbol> output = Optional.empty();

        if (this.getBlock(AllPredefinedLayers.In).isPresent()) {
            input = this.getBlock(AllPredefinedLayers.In);
        }
        if (this.getBlock(AllPredefinedLayers.Out).isPresent()) {
            output = this.getBlock(AllPredefinedLayers.Out);
        }
        ArchitectureElementSymbol block = this.getBlock(AllPredefinedLayers.Block).get();

        if (input.isPresent()) {
            connectBlock(input.get(), Ada, Ada.getInputElement().get(), block);
        } else {
            input = Ada.getInputElement();
        }
        if (output.isPresent()) {
            connectBlock(output.get(), Ada, block, Ada.getOutputElement().get());
        } else {
            output = Ada.getOutputElement();
        }
        connectBlock(block, Ada, input.get(), output.get());
    }

    private void buildBlock(String target) {
        // builds the block inBlock,outBlock or BuildingBlock
        String blockName = getName(target).get();

        LayerSymbol.Builder blockBuilder = new LayerSymbol.Builder();
        LayerDeclarationSymbol declaration;
        Collection<LayerDeclarationSymbol> declarationCollection = getEnclosingScope().resolveMany(blockName, LayerDeclarationSymbol.KIND);

        if (!declarationCollection.isEmpty()) {
            declaration = declarationCollection.iterator().next();
            blockBuilder.declaration(declaration);
            ArgumentSymbol.Builder argBuilder = new ArgumentSymbol.Builder();
            List<ArgumentSymbol> args = new ArrayList<>();
            for (ParameterSymbol param : declaration.getParameters()) {
                // if no Block parameter is passed default to a FullyConnected Layer
                // and set the a default Value
                ArgumentSymbol arg;
                if (param.getName().equals(AllPredefinedLayers.UNITS_NAME)
                        && target.equals(AllPredefinedLayers.Block)
                        && declaration.getName().equals(AllPredefinedLayers.FULLY_CONNECTED_NAME)
                ) {
                    arg = argBuilder.parameter(param.getName()).parameter(param).value(ArchSimpleExpressionSymbol.of(AllPredefinedLayers.DEFAULT_UNITS)).build();
                } else {
                    arg = argBuilder.parameter(param.getName()).parameter(param).value(param.getExpression()).build();
                }
                args.add(arg);
            }
            blockBuilder.arguments(args);
        }

        LayerSymbol block = blockBuilder.build();

        block.setEnclosingScope(this.getSpannedScope());
        setBlock(target, Optional.of(block));
    }

    @Override
    public List<ArchTypeSymbol> computeOutputTypes(List<ArchTypeSymbol> inputTypes, LayerSymbol layer, VariableSymbol.Member member) {
        this.setBlock_name(layer.getStringValue(AllPredefinedLayers.Block)); // get Argument Value of block parameter
        this.setIn_name(layer.getStringValue(AllPredefinedLayers.In));       // get Argument Value of In parameter
        this.setOut_name(layer.getStringValue(AllPredefinedLayers.Out));     // get Argument Value of Out parameter

        if (!getName(AllPredefinedLayers.In).get().equals(AllPredefinedLayers.DEFAULT_BLOCK) || getEnclosingScope().getLocalSymbols().containsKey(getName(AllPredefinedLayers.In).get())) { //passed Parameter is not the default value
            buildBlock(AllPredefinedLayers.In);
        }
        if (getName(AllPredefinedLayers.Block).get().equals(AllPredefinedLayers.DEFAULT_BLOCK) && !getEnclosingScope().getLocalSymbols().containsKey(getName(AllPredefinedLayers.Block).get())) {
            this.setBlock_name(Optional.of(AllPredefinedLayers.FULLY_CONNECTED_NAME));
        }
        buildBlock(AllPredefinedLayers.Block);
        if (!getName(AllPredefinedLayers.Out).get().equals(AllPredefinedLayers.DEFAULT_BLOCK) || getEnclosingScope().getLocalSymbols().containsKey(getName(AllPredefinedLayers.Out).get())) { //passed Parameter is not the default value
            buildBlock(AllPredefinedLayers.Out);
        }
        connectAdaNet(layer);
        getBlock(AllPredefinedLayers.Block).ifPresent(toSolve -> {
            try {
                toSolve.resolve();
            } catch (ArchResolveException e) {
                e.printStackTrace();
            }
        });
        getBlock(AllPredefinedLayers.In).ifPresent(toSolve -> {
            try {
                toSolve.resolve();
            } catch (ArchResolveException e) {
                e.printStackTrace();
            }
        });
        getBlock(AllPredefinedLayers.Out).ifPresent(toSolve -> {
            try {
                toSolve.resolve();
            } catch (ArchResolveException e) {
                e.printStackTrace();
            }
        });
        try {
            layer.resolve();
        }catch(ArchResolveException e){
            e.printStackTrace();
        }
        if(this.getBlock(AllPredefinedLayers.Out).isPresent()){
            return ((LayerSymbol) this.getBlock(AllPredefinedLayers.Out).get()).computeOutputTypes();
        }else {
            return ((LayerSymbol) this.getBlock(AllPredefinedLayers.Block).get()).computeOutputTypes();
        }
    }

    public static AdaNet create() {
        AdaNet declaration = new AdaNet();
        List<ParameterSymbol> parameters = new ArrayList<>(Arrays.asList(
                new ParameterSymbol
                        .Builder()
                        .name(AllPredefinedLayers.Block)
                        //.defaultValue(AllPredefinedLayers.FULLY_CONNECTED_NAME)
                        .defaultValue(AllPredefinedLayers.DEFAULT_BLOCK)
                        .build(),
                new ParameterSymbol
                        .Builder()
                        .name(AllPredefinedLayers.In)
                        .defaultValue(AllPredefinedLayers.DEFAULT_BLOCK)
                        .build(),
                new ParameterSymbol
                        .Builder()
                        .name(AllPredefinedLayers.Out)
                        .defaultValue(AllPredefinedLayers.DEFAULT_BLOCK)
                        .build()
        )
        );

        declaration.setParameters(parameters);
        return declaration;
    }
}
