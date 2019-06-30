package de.monticore.lang.monticar.generator.mqtt.template;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;

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
	
	public void addPorts(Collection<EMAPortInstanceSymbol> ports) 
	{
		for (EMAPortInstanceSymbol port : ports)
		{
			String kind = port.isIncoming()?"incoming":"outgoing";
			this.ports.add(port.getName()+" : "+kind);
		}
    }
	
	public List<String> getPorts()
	{
		return this.ports;
	}
}
