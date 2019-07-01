package de.monticore.lang.monticar.generator.mqtt.template;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol.MqttConnectionKind;

import java.util.*;

// Used to fill .ftl files

public class MqttAdapterModel {
	
	private String compName;
	private List<String> ports = new ArrayList<>();
	
	public MqttAdapterModel(String compName) 
	{
        this.compName = compName;
    }
	
	public String getCompName()
	{
		return compName;
	}
	
	// Parse through component to find information about its ports
	public void addPorts(Collection<EMAPortInstanceSymbol> ports) 
	{
		for (EMAPortInstanceSymbol port : ports)
		{
			String kind = port.isIncoming()?"incoming":"outgoing";
			String symbolKind = "unknown symbol";
			Optional<MiddlewareSymbol> symbol = port.getMiddlewareSymbol();
			if(symbol.isPresent() && symbol.get().isKindOf(MqttConnectionKind.INSTANCE))
			{
				MqttConnectionSymbol sym = (MqttConnectionSymbol) symbol.get();
				String topicName = sym.getTopicName().isPresent()?sym.getTopicName().get():"unknown";
				symbolKind = "mqtt, topic: "+topicName;
			}
			
			this.ports.add(port.getName()+" : "+kind+" ("+symbolKind+")");
		}
    }
	
	public List<String> getPorts()
	{
		return this.ports;
	}
}
