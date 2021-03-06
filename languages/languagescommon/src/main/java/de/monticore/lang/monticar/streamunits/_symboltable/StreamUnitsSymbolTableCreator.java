/* (c) https://github.com/MontiCore/monticore */
/* generated from model null*/
/* generated by template symboltable.SymbolTableCreator*/


package de.monticore.lang.monticar.streamunits._symboltable;

import de.monticore.lang.monticar.streamunits._ast.*;
import de.monticore.literals.literals._ast.ASTSignedLiteral;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class StreamUnitsSymbolTableCreator extends StreamUnitsSymbolTableCreatorTOP {

    private static int id = 0;

    public StreamUnitsSymbolTableCreator(
            final ResolvingConfiguration resolvingConfig, final MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    @Override
    public void visit(ASTStreamUnitsCompilationUnit node) {
        Log.debug("Building Symboltable for Stream: " + node.getComponentStreamUnits().getName(),
                StreamUnitsSymbolTableCreator.class.getSimpleName());
        String compilationUnitPackage = Names.getQualifiedName(node.getPackageList());
        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                compilationUnitPackage,
                new ArrayList<>());
        putOnStack(artifactScope);
        id++;
    }

    @Override
    public void endVisit(ASTStreamUnitsCompilationUnit node) {
        removeCurrentScope();
    }

    @Override
    public void visit(ASTComponentStreamUnits node) {
        ComponentStreamUnitsSymbol streamSymbol = new ComponentStreamUnitsSymbol(node.getName());
        addToScopeAndLinkWithNode(streamSymbol, node);
    }

    @Override
    public void endVisit(ASTComponentStreamUnits node) {
        removeCurrentScope();
    }

    @Override
    public void visit(ASTNamedStreamUnits node) {
        String qualifiedName;
        if(node.getFieldQualifierList().isEmpty()) {
            qualifiedName = node.getName();
        }else{
            qualifiedName = node.getName() + "." + String.join(".", node.getFieldQualifierList());
        }
        NamedStreamUnitsSymbol streamSymbol = new NamedStreamUnitsSymbol(qualifiedName, id);
        for (ASTStreamInstruction streamInstruction : node.getStream().getStreamInstructionList()) {
            if (streamInstruction.getStreamValueOpt().isPresent()) {
                streamSymbol.add(handleStreamValue(streamInstruction.getStreamValueOpt().get()));
            } else if (streamInstruction.getStreamCompareOpt().isPresent()) {
                ASTStreamCompare astStreamCompare = streamInstruction.getStreamCompareOpt().get();
                streamSymbol.add(new StreamCompare(new StreamValuePrecision(astStreamCompare.getLeft()),
                        astStreamCompare.getOperator(), new StreamValuePrecision(astStreamCompare.getRight())));
            } else if (streamInstruction.getStreamArrayValuesOpt().isPresent()) {
                streamSymbol.add(handleStreamArrayValues(streamInstruction));
            }
        }
        addToScopeAndLinkWithNode(streamSymbol, node);
    }

    private IStreamValue handleStreamValue(ASTStreamValue streamValue) {
        IStreamValue result = null;
        if (streamValue.getPrecisionNumberOpt().isPresent()) {
            ASTPrecisionNumber num = streamValue.getPrecisionNumberOpt().get();
            if (num.getPrecisionOpt().isPresent()) {
                result = new StreamValuePrecision(num.getNumberWithUnit(), num.getPrecisionOpt().get().getNumberWithUnit());
            } else {
                result = (new StreamValuePrecision(num.getNumberWithUnit()));
            }
        } else if (streamValue.getNameOpt().isPresent()) {
            result = (new StreamValuePrecision(streamValue.getNameOpt().get()));
        } else if (streamValue.getSignedLiteralOpt().isPresent()) {
            ASTSignedLiteral signedLiteral = streamValue.getSignedLiteralOpt().get();
            result = (new StreamValuePrecision(signedLiteral));
        } else if (streamValue.getDontCareOpt().isPresent()) {
            result = (new StreamValuePrecision("-"));
        } else if (streamValue.getValueAtTickOpt().isPresent()) {
            ASTValueAtTick valueAtTick = streamValue.getValueAtTickOpt().get();
            result = (new StreamValueAtTick(valueAtTick));
        }
        return result;
    }

    private StreamValues handleStreamArrayValues(ASTStreamInstruction streamInstruction) {
        ASTStreamArrayValues streamArrayValues = streamInstruction.getStreamArrayValues();
        StreamValues result = null;
        if (streamArrayValues.getMatrixPairOpt().isPresent()) {
            result = handleMatrixPair(streamArrayValues.getMatrixPairOpt().get());
        } else if (streamArrayValues.getValuePairOpt().isPresent()) {
            result = new StreamValues(handleValuePair(streamArrayValues.getValuePairOpt().get()));
        }
        return result;
    }

    private StreamValues handleMatrixPair(ASTMatrixPair matrixPair) {
        //handle all rows
        StreamValues streamValues = new StreamValues();
        for (int row = 0; row < matrixPair.getValuePairList().size(); ++row) {
            streamValues.add(handleValuePair(matrixPair.getValuePairList().get(row)));
        }
        return streamValues;
    }

    private List<IStreamValue> handleValuePair(ASTValuePair valuePair) {
        List<ASTStreamValue> streamValues = valuePair.getStreamValueList();
        List<IStreamValue> currentList = new ArrayList<>();
        //handle the elements of each row
        for (int i = 0; i < streamValues.size(); ++i) {
            currentList.add(handleStreamValue(streamValues.get(i)));
        }
        return currentList;
    }
}
