/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.testing;

import de.monticore.ast.ASTNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTPort;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.monticar.common2._ast.ASTCommonMatrixType;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.util.Collection;

/**
 */
public class StreamTestGenerator {
    protected StreamTest currentGeneratedStreamTest;

    int currentPrecisionValueType = 0;// 0: double, 1:int, 2:Boolean
    double doubleMin, doubleMax;
    int intMin, intMax;

    public void createStreamTest(EMAComponentInstanceSymbol expandedComponentInstanceSymbol, int amountOfValues, String testNamePostFix) {

        currentGeneratedStreamTest = new StreamTest();
        Collection<EMAPortInstanceSymbol> inPorts = expandedComponentInstanceSymbol.getIncomingPortInstances();
        currentGeneratedStreamTest.setComponentName(StringUtils.capitalize(expandedComponentInstanceSymbol.getName()));
        currentGeneratedStreamTest.setPackageName(expandedComponentInstanceSymbol.getPackageName());
        currentGeneratedStreamTest.setName(currentGeneratedStreamTest.getComponentName() + "Test" + testNamePostFix);

        Log.debug("Generating StreamTest for component " + currentGeneratedStreamTest.getComponentName(), "StreamTestGenerator.createStreamTest");
        for (EMAPortInstanceSymbol p : inPorts) {
            if (p.getAstNode().isPresent()) {
//                System.out.println(p.getAstNode().get().getClass().getName());
                ASTPort astPort = (ASTPort) p.getAstNode().get();
//                System.out.println(astPort.getName());
                StreamTestPort streamTestPort = new StreamTestPort();
                streamTestPort.setName(p.getName());
                doubleMin = -10000000;//Maybe these values should be configurable as parameters later?
                doubleMax = 10000000;
                intMin = -10000000;
                intMax = 10000000;

                int sizeX = 1;
                int sizeY = 1;
                int type = 0;// 0: singular value, 1: matrix, 2:Boolean
                if (astPort.getType() instanceof ASTElementType) {
                    ASTElementType astElementType = (ASTElementType) astPort.getType();
                    handleASTElementType(astElementType);
                } else if (astPort.getType() instanceof ASTCommonMatrixType) {
                    type = 1;
                    ASTCommonMatrixType astCommonMatrixType = (ASTCommonMatrixType) astPort.getType();
//                    System.out.println(astCommonMatrixType.getDimension().getDimensionList().get(0).getClass().getName());
//                    System.out.println(astCommonMatrixType.getDimension().getDimensionList().get(1).getClass().getName());
                    //System.out.println(astCommonMatrixType.getDimension().getDimensionList().get(0).toString());
                    //System.out.println(astCommonMatrixType.getDimension().getDimensionList().get(1).toString());
                    ASTNode node1 = astCommonMatrixType.getDimension().getDimensionList().get(0);
                    ASTNode node2 = astCommonMatrixType.getDimension().getDimensionList().get(1);

                    if (node1 instanceof ASTNumberExpression) {
                        sizeX = ((ASTNumberExpression) node1).getNumberWithUnit().getNumber().get().intValue();
                    }
                    if (node2 instanceof ASTNumberExpression) {
                        sizeY = ((ASTNumberExpression) node2).getNumberWithUnit().getNumber().get().intValue();
                    }
                    handleASTElementType(astCommonMatrixType.getElementType());
                    //sizeX = astCommonMatrixType.getDimension().getDimensionList().get(0).getClass().getName().toString();
                    //sizeY = astCommonMatrixType.getDimension().getDimensionList().get(1).toString();
                }
//                System.out.println("AmountOfValues:" + amountOfValues);
                for (int i = 0; i < amountOfValues; ++i) {
                    StreamTestValue streamTestValue;
                    if (type == 0) {//singular value
                        if (currentPrecisionValueType == 0) {
                            streamTestValue = new BasicStreamTestValue<Double>(getRandomDoubleBetween(doubleMin, doubleMax));
                        } else if (currentPrecisionValueType == 1) {
                            streamTestValue = new BasicStreamTestValue<Integer>(getRandomIntegerBetween(intMin, intMax));
                        } else /*(currentPrecisionValueType == 2)*/ {
                            streamTestValue = new BasicStreamTestValue<Boolean>(Math.random() > 0.5 ? true : false);
                        }
                    } else if (type == 1) {//matrix value
                        if (currentPrecisionValueType == 0)
                            streamTestValue = new MatrixStreamTestValue<Double>(getRandomDoubleMatrix(sizeX, sizeY, doubleMin, doubleMax));
                        else if (currentPrecisionValueType == 1)
                            streamTestValue = new MatrixStreamTestValue<Integer>(getRandomIntegerMatrix(sizeX, sizeY, intMin, intMax));
                        else
                            streamTestValue = null;//TODO handle Boolean Matrix
                    } else if (type == 2) {//singular boolean value
                        streamTestValue = new BasicStreamTestValue<Boolean>(Math.random() > 0.5 ? true : false);
                    } else {
                        streamTestValue = null;
                        Log.debug("PortType Not Handled!", "StreamTestTestGeneration:");
                    }
                    streamTestPort.addValue(streamTestValue);
                }

                currentGeneratedStreamTest.addPort(streamTestPort);
            } else {
                //ConstantPorts do not have an ASTNode
                //Log.debug(p.getName(), "PortSymbol has no ASTNode:");
            }
//            System.out.println();
        }
        currentGeneratedStreamTest.fixPortArrays();
    }

    private void handleASTElementType(ASTElementType astElementType) {
        if (astElementType.getName().equals("Q")) {
            currentPrecisionValueType = 0;
        } else if (astElementType.getName().equals("Z")) {
            currentPrecisionValueType = 1;
        } else if (astElementType.getName().equals("N")) {
            currentPrecisionValueType = 1;
        } else if (astElementType.getName().equals("B")) {
            currentPrecisionValueType = 2;
        }
        if (astElementType.isPresentRange()) {
            if (astElementType.getRange().getMin().getNumber().isPresent()) {
                doubleMin = astElementType.getRange().getStartValue().doubleValue();
                intMin = astElementType.getRange().getStartValue().intValue();
            }
//            System.out.println("Min: " + doubleMin);
            if (astElementType.getRange().getMax().getNumber().isPresent()) {
                doubleMax = astElementType.getRange().getEndValue().doubleValue();
                intMax = astElementType.getRange().getEndValue().intValue();
//                System.out.println("Max: " + doubleMax);
            }
        }
    }

    public StreamTest getCurrentGeneratedStreamTest() {
        return currentGeneratedStreamTest;
    }

    public int getRandomIntegerBetween(int min, int max) {
        return (int) getRandomDoubleBetween(min, max);
    }

    public Integer[][] getRandomIntegerMatrix(int sizeX, int sizeY) {
        return getRandomIntegerMatrix(sizeX, sizeY, Integer.MIN_VALUE, Integer.MAX_VALUE);
    }

    public Integer[][] getRandomIntegerMatrix(int sizeX, int sizeY, int min, int max) {
        Integer[][] matrix = new Integer[sizeX][sizeY];
        for (int i = 0; i < sizeX; ++i)
            for (int j = 0; j < sizeY; ++j)
                matrix[i][j] = getRandomIntegerBetween(min, max);
        return matrix;
    }

    public double getRandomDoubleBetween(double min, double max) {
        return Math.random() + min + Math.random() * (max - min);
    }

    public Double[][] getRandomDoubleMatrix(int sizeX, int sizeY, double min, double max) {
        Double[][] matrix = new Double[sizeX][sizeY];
//        System.out.println("Started Generation of random " + sizeX + "x" + sizeY + " matrix");
        for (int i = 0; i < sizeX; ++i)
            for (int j = 0; j < sizeY; ++j)
                matrix[i][j] = getRandomDoubleBetween(min, max);
//        System.out.println("Generation of this matrix is done");
        return matrix;
    }

    public Double[][] getRandomDoubleMatrix(int sizeX, int sizeY) {
        return getRandomDoubleMatrix(sizeX, sizeY, Double.MIN_VALUE, Double.MAX_VALUE);
    }
}
