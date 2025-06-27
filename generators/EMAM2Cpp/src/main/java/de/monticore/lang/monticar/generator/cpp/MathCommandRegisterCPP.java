/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.MathCommandRegister;
import de.monticore.lang.monticar.generator.cpp.commands.*;
import de.se_rwth.commons.logging.Log;

/**
 */
public class MathCommandRegisterCPP extends MathCommandRegister {

    @Override
    protected void init() {

        //registerMathCommand("size", "testo");
        registerMathCommand(new MathScaleCubeCommand());
        registerMathCommand(new MathJoinCubeDimCommand());
        registerMathCommand(new MathAtan2Command());
        registerMathCommand(new MathLog2Command());
        registerMathCommand(new MathSizeCommand());
        registerMathCommand(new MathSumCommand());
        registerMathCommand(new MathEyeCommand());
        registerMathCommand(new MathNormCommand());
        registerMathCommand(new MathDiagCommand());
        registerMathCommand(new MathEigvalCommand());
        registerMathCommand(new MathEigvecCommand());
        registerMathCommand(new MathGcdCommand());
        registerMathCommand(new MathInvCommand());
        registerMathCommand(new MathInvDiagCommand());
        registerMathCommand(new MathAbsCommand());
        registerMathCommand(new MathAcosCommand());
        registerMathCommand(new MathAcoshCommand());
        registerMathCommand(new MathAsinCommand());
        registerMathCommand(new MathAsinhCommand());
        registerMathCommand(new MathAtanCommand());
        registerMathCommand(new MathAtanhCommand());
        registerMathCommand(new MathCosCommand());
        registerMathCommand(new MathCoshCommand());
        registerMathCommand(new MathExpCommand());
        registerMathCommand(new MathLogCommand());
        registerMathCommand(new MathLog10Command());
        registerMathCommand(new MathSinCommand());
        registerMathCommand(new MathSinhCommand());
        registerMathCommand(new MathSqrtCommand());
        registerMathCommand(new MathTanCommand());
        registerMathCommand(new MathTanhCommand());
        registerMathCommand(new MathMinCommand());
        registerMathCommand(new MathMaxCommand());
        registerMathCommand(new MathOnesCommand());
        registerMathCommand(new MathZerosCommand());
        registerMathCommand(new MathDetCommand());
        registerMathCommand(new MathKMeansCommand());
        registerMathCommand(new MathSqrtmCommand());
        registerMathCommand(new MathSqrtmDiagCommand());
        registerMathCommand(new MathCeilCommand());
        registerMathCommand(new MathPowCommand());

        //register CV commands
        registerMathCommand(new ErodeCommand());
        registerMathCommand(new DilateCommand());
        registerMathCommand(new BoundingRectCommand());
        registerMathCommand(new CvtColorCommand());
        registerMathCommand(new FindContoursCommand());
        registerMathCommand(new GaussianBlurCommand());
        registerMathCommand(new InRangeCommand());
        registerMathCommand(new LargestContourCommand());
        registerMathCommand(new RectangleCommand());


        //for fixing some errors
        registerMathCommand(new MathRowCommand());
        registerMathCommand(new MathColumnCommand());

        //for dynamic
        registerMathCommand(new DynamicMathPortIsConnectedCommand());
        registerMathCommand(new DynamicMathPortNewConnectCommand());
        registerMathCommand(new DynamicMathPortFreeCommand());
    }

    /**
     * Set input to converted string of mathExpressionSymbol.getTextualRepresentation()
     *
     * @param mathExpressionSymbol
     * @param input
     * @return
     */
    public static boolean containsCommandExpression(MathExpressionSymbol mathExpressionSymbol, String input) {
        mathExpressionSymbol = mathExpressionSymbol.getRealMathExpressionSymbol();
        Log.info("trying containsCommand " + input + "class: " + mathExpressionSymbol.getClass().getName(), "Info");

        try {
            //if ((mathExpressionSymbol.isMatrixExpression() && ((MathMatrixExpressionSymbol) mathExpressionSymbol).isMatrixNameExpression()))
            {
                //MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol = (MathMatrixNameExpressionSymbol) mathExpressionSymbol;
                //String fullName = mathMatrixNameExpressionSymbol.getTextualRepresentation();
                String fullName = input;
                while (fullName.length() > 0) {
                    fullName = removeTrailingStrings(fullName, "(");
                    String name = calculateName(fullName);
                    Log.info("" + input + " name: " + name, "containsCommandExpression");
                    if (GeneratorCPP.currentInstance.getMathCommandRegister().isMathCommand(name)) {
                        return true;
                    }
                    fullName = fullName.substring(name.length() + 1);
                }
            }
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        return false;
    }

    public static String removeTrailingStrings(String fullString, String firstStringParts) {
        while (fullString.startsWith(firstStringParts)) {
            fullString = fullString.substring(1);
        }
        return fullString;
    }

    public static String removeBrackets(String fullString) {
        String result = removeTrailingStrings(fullString, "(");
        int idx = result.indexOf("(");
        if (idx > 0)
            result = result.substring(0, idx);
        return result;
    }

    public static String calculateName(String fullName) {
        int index = fullName.indexOf("(");
        String name = "";
        if (index != -1) {
            name = fullName.substring(0, index);
        }
        if (name.contains(".")) {
            Log.info(name, "Splitting");
            name = name.split("\\.")[1];
        }
        return name;
    }
}
