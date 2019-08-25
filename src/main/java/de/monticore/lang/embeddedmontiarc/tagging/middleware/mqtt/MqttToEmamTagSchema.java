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
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class MqttToEmamTagSchema {

    protected static MqttToEmamTagSchema instance = null;

    protected MqttToEmamTagSchema() {
    }

    protected static MqttToEmamTagSchema getInstance() {
        if (instance == null) {
            instance = new MqttToEmamTagSchema();
        }
        return instance;
    }

    protected void doRegisterTagTypes(TaggingResolver tagging) {
        tagging.addTagSymbolCreator(new MqttConnectionSymbolCreator());
        tagging.addTagSymbolResolvingFilter(CommonResolvingFilter.create(MqttConnectionSymbol.KIND));
    }

    public static void registerTagTypes(TaggingResolver tagging) {
        getInstance().doRegisterTagTypes(tagging);
    }
}
