/* (c) https://github.com/MontiCore/monticore */
//package de.monticore.lang.montiarc;
//
//import de.monticore.ModelingLanguageFamily;
//import de.monticore.io.paths.ModelPath;
//import de.monticore.lang.montiarc.math.Calculator;
//import ASTMathScript;
//import MathCoCoChecker;
//import MathParser;
//import MathLanguage;
//import MathSymbolTableCreator;
//import MathVariableDeclarationSymbol;
//import de.monticore.symboltable.GlobalScope;
//import de.monticore.symboltable.ResolvingConfiguration;
//import de.monticore.symboltable.Scope;
//import org.antlr.v4.runtime.RecognitionException;
//import org.junit.Test;
//
//import java.io.IOException;
//import java.nio.file.Path;
//import java.nio.file.Paths;
//import java.util.Optional;
//
//import static org.junit.Assert.assertNotNull;
//
///**
// * Created by Tobias PC on 04.01.2017.
// */
//public class CalculationTest extends AbstractMathChecker {
//    @Test
//    public void test1() throws IOException{
//        String model = "src/test/resources/Calculations/example2.m";
//        ASTMathScript root = loadModel(model);
//        Calculator.calculate(root);
//    }
//
//    @Override
//    protected MathCoCoChecker getChecker() {
//        return null;
//    }
//}
