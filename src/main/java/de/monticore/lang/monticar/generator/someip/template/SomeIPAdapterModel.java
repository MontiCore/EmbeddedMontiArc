/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.someip.template;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol.SomeIPConnectionKind;

import java.util.*;
import java.util.stream.Collectors;

// Used to fill .ftl files

public class SomeIPAdapterModel {

	private String compName;
	private List<String> ports = new ArrayList<>();
	private List<EMAPortInstanceSymbol> incoming = new ArrayList<>();
	private List<EMAPortInstanceSymbol> outgoing = new ArrayList<>();

	public SomeIPAdapterModel(String compName) {
        this.compName = compName;
    }

	public String getCompName()	{
		return compName;
	}

	public String getEscapedCompName() 	{
		return compName
				.replace('.', '_')
				.replace('[', '_')
				.replace(']', '_');
	}

	public List<EMAPortInstanceSymbol> getIncomingPorts() {
		return incoming;
	}

	public List<EMAPortInstanceSymbol> getOutgoingPorts() {
		return outgoing;
	}

	public void addPorts(Collection<EMAPortInstanceSymbol> ports) {
		incoming.addAll(ports);
		incoming = incoming.stream().filter(fc -> fc.isSomeIPPort()).filter(fc -> fc.isIncoming()).collect(Collectors.toList());

		outgoing.addAll(ports);
		outgoing = outgoing.stream().filter(fc -> fc.isSomeIPPort()).filter(fc -> fc.isOutgoing()).collect(Collectors.toList());
	}

	public int getServiceId(EMAPortInstanceSymbol p)
	{
		Optional<MiddlewareSymbol> symbol = p.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
		{
			SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
			int serviceID = sym.getserviceID().isPresent()?sym.getserviceID().get():-1;
			return serviceID;
		}
		return -1;
	}

	public int getInstanceId(EMAPortInstanceSymbol p)
	{
		Optional<MiddlewareSymbol> symbol = p.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
		{
			SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
			int instanceID = sym.getinstanceID().isPresent()?sym.getinstanceID().get():-1;
			return instanceID;
		}
		return -1;
	}

	public int getEventGroupId(EMAPortInstanceSymbol p)
	{
		Optional<MiddlewareSymbol> symbol = p.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
		{
			SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
			int eventgroupID = sym.geteventgroupID().isPresent()?sym.geteventgroupID().get():-1;
			return eventgroupID;
		}
		return -1;
	}

	public int getEventId(EMAPortInstanceSymbol p)
	{
		Optional<MiddlewareSymbol> symbol = p.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
		{
			SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
			int eventID = 0;//sym.geteventID().isPresent()?sym.geteventID().get():-1;
			return eventID;
		}
		return -1;
	}

	public int getMethodId(EMAPortInstanceSymbol p)
	{
		Optional<MiddlewareSymbol> symbol = p.getMiddlewareSymbol();
		if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
		{
			SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
			int methodID = 0;//sym.getmethodID().isPresent()?sym.getmethodID().get():-1;
			return methodID;
		}
		return -1;
	}

	// Parse through component to find information about its ports
	public void addPortsDesc(Collection<EMAPortInstanceSymbol> ports)
	{
		for (EMAPortInstanceSymbol port : ports)
		{
			String kind = port.isIncoming()?"incoming":"outgoing";
			String symbolKind = "unknown symbol";
			Optional<MiddlewareSymbol> symbol = port.getMiddlewareSymbol();
			if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
			{
				SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
				int serviceID = sym.getserviceID().isPresent()?sym.getserviceID().get():-1;
				int instanceID = sym.getinstanceID().isPresent()?sym.getinstanceID().get():-1;
				int eventgroupID = sym.geteventgroupID().isPresent()?sym.geteventgroupID().get():-1;
				symbolKind = "someip, serviceID: " + serviceID + " instanceID: " + instanceID + " eventgroupID: " + eventgroupID;
			}

			this.ports.add(port.getName() + " : " + kind + " (" + symbolKind + ")");
		}
    }

	public List<String> getPortsDesc()
	{
		return this.ports;
	}

}
