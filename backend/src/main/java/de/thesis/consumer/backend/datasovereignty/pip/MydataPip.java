package de.thesis.consumer.backend.datasovereignty.pip;

import de.fraunhofer.iese.mydata.pip.PipService;
import de.fraunhofer.iese.mydata.registry.ActionDescription;
import de.fraunhofer.iese.mydata.registry.ActionParameterDescription;

@PipService(componentName = "authoritypip")
public class MydataPip {
	@ActionDescription
	public String getAuthority(
			@ActionParameterDescription(name = "username", mandatory = true) String username
	) {
		if (username.equals("DHL Western Germany Trucks")) {
			return "success";
		}

		return "failure";
	}
}
