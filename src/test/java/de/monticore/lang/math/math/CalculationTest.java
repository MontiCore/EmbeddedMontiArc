/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
