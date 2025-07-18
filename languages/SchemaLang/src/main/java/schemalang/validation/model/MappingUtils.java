package schemalang.validation.model;

import com.google.common.collect.Lists;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberExpression;
import de.monticore.lang.monticar.ts.MCASTTypeSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.CommonMCTypeReference;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.monticore.lang.monticar.types2._ast.ASTDimension;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.literals.literals._ast.ASTIntLiteral;
import de.monticore.numberunit._ast.ASTNumberWithInf;
import de.monticore.numberunit._ast.ASTNumberWithUnit;
import de.monticore.types.types._ast.ASTType;

import java.math.BigDecimal;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class MappingUtils {

    public static ArchitectureComponent createArchitectureComponent(EMAComponentSymbol emaComponent) {
        ArchitectureComponent architectureComponent = new ArchitectureComponent();
        architectureComponent.setFullName(emaComponent.getFullName());
        architectureComponent.setPorts(mapPorts(emaComponent.getPortsList()));
        return architectureComponent;
    }

    private static List<Port> mapPorts(Collection<EMAPortSymbol> emaPorts) {
        if (emaPorts == null || emaPorts.isEmpty()) {
            return Lists.newArrayList();
        }

        List<Port> ports = Lists.newArrayList();
        for (EMAPortSymbol emaPort : emaPorts) {
            ports.add(mapPort(emaPort));
        }
        return ports;
    }

    private static Port mapPort(EMAPortSymbol emaPort) {
        Port port = new Port();
        if (emaPort.isIncoming()) {
            port.setPortDirection(PortDirection.INPUT);
        } else if (emaPort.isOutgoing()) {
            port.setPortDirection(PortDirection.OUTPUT);
        }
        port.setPortName(emaPort.getName());
        port.setPortType(mapPortType(emaPort.getTypeReference()));
        return port;
    }

    private static PortType mapPortType(MCTypeReference<? extends MCTypeSymbol> typeReference) {
        PortType portType = new PortType();
        if (typeReference instanceof MCASTTypeSymbol) {
            MCASTTypeSymbol typeSymbol = (MCASTTypeSymbol) typeReference;
            ASTType astType = typeSymbol.getAstType();
            if (astType instanceof ASTCommonMatrixType) {
                ASTCommonMatrixType commonMatrixType = (ASTCommonMatrixType) astType;
                ASTElementType elementType = commonMatrixType.getElementType();
                portType.setTypeIdentifier(mapTypeIdentifier(elementType));

                Optional<ASTRange> astRangeOpt = elementType.getRangeOpt();
                if (astRangeOpt.isPresent()) {
                    ASTRange astRange = astRangeOpt.get();
                    portType.setRange(mapRange(astRange));
                }

                ASTDimension dimension = commonMatrixType.getDimension();
                if (dimension != null && dimension.getDimensionList() != null
                        && !dimension.getDimensionList().isEmpty()) {
                    portType.setDimension(mapDimension(dimension));
                }
            }

        } else if (typeReference instanceof CommonMCTypeReference) {
            CommonMCTypeReference mcTypeReference = (CommonMCTypeReference) typeReference;
            portType.setTypeIdentifier(mcTypeReference.getName());
        }
        return portType;
    }

    private static Dimension mapDimension(ASTDimension emaDimension) {
        Dimension dimension = new Dimension();
        List<Integer> dimensionList = Lists.newArrayList();
        List<ASTExpression> astExpressions = emaDimension.getDimensionList();
        for (ASTExpression astExpression : astExpressions) {
            ASTNumberWithUnit numberWithUnit;
            if (astExpression instanceof ASTNumberExpression) {
                ASTNumberExpression expression = (ASTNumberExpression) astExpression;
                numberWithUnit = expression.getNumberWithUnit();
            } else {
                ASTUnitNumberExpression expression = (ASTUnitNumberExpression) astExpression;
                numberWithUnit = expression.getNumberWithUnit();
            }
            ASTNumberWithInf numberWithInf = numberWithUnit.getNum();
            ASTIntLiteral intLiteral = (ASTIntLiteral) numberWithInf.getNumber();
            dimensionList.add(intLiteral.getValue());
        }
        dimension.setDimensionList(dimensionList);
        dimension.setSize(0);
        return dimension;
    }

    private static String mapTypeIdentifier(ASTElementType elementType) {
        return elementType.getName();
    }

    private static Range mapRange(ASTRange astRange) {
        if (astRange.hasNoLowerLimit() || astRange.hasNoUpperLimit()) {
            return null;
        }
        Range range = new Range();
        range.setStartValue(BigDecimal.valueOf(astRange.getStartValue().doubleValue()));
        range.setEndValue(BigDecimal.valueOf(astRange.getEndValue().doubleValue()));
        return range;
    }
}