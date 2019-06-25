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
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.tagging._symboltable.TagKind;
import de.monticore.lang.tagging._symboltable.TagSymbol;

import java.util.Optional;

public class DataPathSymbol extends TagSymbol {
    public static final DataPathKind KIND  = DataPathKind.INSTANCE;

    public DataPathSymbol() {
        super(KIND, ".");
    }

    public DataPathSymbol(String path, String type) {
        this(KIND, path, type);
    }

    public DataPathSymbol(DataPathKind kind, String path, String type) {
        super(kind, path, type);
    }

    public String getPath() {
      return getValue(0);
    }

    public String getType() {
      return getValue(1);
    }

    @Override
    public String toString() {
        return super.toString();
    }

    public static class DataPathKind extends TagKind {
        public static final DataPathKind INSTANCE = new DataPathKind();

        protected DataPathKind() {
        }
    }
}
