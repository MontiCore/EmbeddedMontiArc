/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EffectorSymbol;
import de.monticore.lang.embeddedmontiview.tagging.LatencyTagSchema.LatencyViewEffSymbol;
import de.monticore.lang.montiarc.tagging._symboltable.TagKind;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.assertTrue;

/**
 * Created by Yannick Deuster on 18.10.17.
 */
public class TaggingTest extends AbstractSymtabTest {
    protected final String resourcePath = "src/test/resources/";

    @Test
    public void testTags_Effector() {
        Scope s = createSymTab( "view.wbView.WCET1", resourcePath);

        Optional<EffectorSymbol> eff = s.resolve("view.wbView.WCET1.WeatherBalloonSensors.controlSignalsIn -> dataSaveInternalOut", EffectorSymbol.KIND);
        assertTrue(eff.isPresent());

        //Collection tag = eff.get().getTags((TagKind) LatencyViewEffSymbol.KIND);
        //assertTrue(tag.size() == 1);
    }
}
