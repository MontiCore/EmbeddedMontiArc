package de.monticore.lang.monticar.generator.someip.template;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.MiddlewareSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol.SomeIPConnectionKind;

import java.util.*;

// Used to fill .ftl files

public class SomeIPAdapterModel {

	private String compName;
	private List<String> ports = new ArrayList<>();

	public SomeIPAdapterModel(String compName) {
        this.compName = compName;
    }

	public String getCompName()	{
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
			if(symbol.isPresent() && symbol.get().isKindOf(SomeIPConnectionKind.INSTANCE))
			{
				SomeIPConnectionSymbol sym = (SomeIPConnectionSymbol) symbol.get();
				int serviceID = sym.getserviceID().isPresent()?sym.getserviceID().get():-1;
				symbolKind = "someip, serviceID: " + serviceID;
			}

			this.ports.add(port.getName() + " : " + kind + " (" + symbolKind + ")");
		}
    }

	public List<String> getPorts()
	{
		return this.ports;
	}

}
