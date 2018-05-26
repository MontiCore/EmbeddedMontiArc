/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt._symboltable;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math.MathSymbolTableCreatorTest;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.junit.BeforeClass;

import java.nio.file.Paths;

/**
 * Tests all Math Symbols with the new SymbolTableCreator.
 * @author Christoph Richter
 */
public class MathOptSymbolTableCreatorTest extends MathSymbolTableCreatorTest {

    @BeforeClass
    public static void setUpClass() {
        // only create symTab once
        symTab = createSymTab("src/test/resources");
    }

    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new MathLanguage());
        fam.addModelingLanguage(new MathOptLanguage());
        final ModelPath mp = new ModelPath();

        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }

}