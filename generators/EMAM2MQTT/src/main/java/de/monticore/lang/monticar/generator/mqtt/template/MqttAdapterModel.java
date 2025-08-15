/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt.template;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol.MqttConnectionKind;

import java.util.*;
import java.util.stream.Collectors;

// Used to fill .ftl files

public class MqttAdapterModel {
	
	private String compName;
	private List<String> ports = new ArrayList<>();
	private List<EMAPortInstanceSymbol> incoming = new ArrayList<>();
	private List<EMAPortInstanceSymbol> outgoing = new ArrayList<>();
	
	public MqttAdapterModel(String compName) 
	{
        this.compName = compName;
    }
	
	public String getCompName()
	{
		return compName;
	}
	
	public String getEscapedCompName()
	{
		return compName
				.replace('.', '_')
				.replace('[', '_')
				.replace(']', '_');
	}
	
	public List<EMAPortInstanceSymbol> getIncomingPorts()
	{
		return incoming;
	}
	
	public List<EMAPortInstanceSymbol> getOutgoingPorts()
	{
		return outgoing;
	}
	
	public void addPorts(Collection<EMAPortInstanceSymbol> ports)
	{
		incoming.addAll(ports);
		incoming = incoming.stream().filter(fc -> fc.isMqttPort()).filter(fc -> fc.isIncoming()).collect(Collectors.toList());
		
		outgoing.addAll(ports);
		outgoing = outgoing.stream().filter(fc -> fc.isMqttPort()).filter(fc -> fc.isOutgoing()).collect(Collectors.toList());
	}
	
	public String getTopic(EMAPortInstanceSymbol port)
	{
		Optional<MiddlewareSymbol> symbol = port.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(MqttConnectionKind.INSTANCE))
		{
			MqttConnectionSymbol sym = (MqttConnectionSymbol) symbol.get();
			String topicName = sym.getTopicName().isPresent()?sym.getTopicName().get():"unknown";
			return topicName;
		}
		return "";
	}
	
	// Parse through component to find information about its ports
	public void addPortsDesc(Collection<EMAPortInstanceSymbol> ports) 
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
	
	public List<String> getPortsDesc()
	{
		return this.ports;
	}
}
