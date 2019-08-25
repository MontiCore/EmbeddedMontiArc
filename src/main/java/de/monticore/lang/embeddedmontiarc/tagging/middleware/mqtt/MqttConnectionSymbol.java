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
package de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt;

import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.tagging._symboltable.TagKind;

import java.util.Optional;

public class MqttConnectionSymbol extends MiddlewareSymbol {

    public static final MqttConnectionKind KIND = MqttConnectionKind.INSTANCE;

    public MqttConnectionSymbol() {
        super(KIND, Optional.empty(), Optional.empty());
    }

    public MqttConnectionSymbol(String topicName) {
        this(KIND, topicName);
    }

    public MqttConnectionSymbol(MqttConnectionKind kind, String topicName) {
        super(kind, Optional.ofNullable(topicName), Optional.empty());
    }

    public MqttConnectionSymbol(String topicName, String msgField) {
        this(KIND, topicName, msgField);
    }

    protected MqttConnectionSymbol(MqttConnectionKind kind, String topicName, String msgField) {
        super(kind, Optional.ofNullable(topicName), Optional.ofNullable(msgField));
    }

    @Override
    public String toString() {
        return String.format("MqttConnection = %s, %s",
                getTopicName(), getMsgField());
    }

    public Optional<String> getTopicName() {
        return getValue(0);
    }

    public void setTopicName(String topicName) {
        this.values.set(0, Optional.ofNullable(topicName));
    }

    public Optional<String> getMsgField() {
        return getValue(1);
    }

    public void setMsgField(String msgField) {
        this.values.set(1, Optional.ofNullable(msgField));
    }

    public static class MqttConnectionKind extends TagKind {
        public static final MqttConnectionKind INSTANCE = new MqttConnectionKind();

        protected MqttConnectionKind() {
        }
    }
}
