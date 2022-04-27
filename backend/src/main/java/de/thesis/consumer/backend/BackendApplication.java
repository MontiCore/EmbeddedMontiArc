package de.thesis.consumer.backend;

import de.fraunhofer.iese.mydata.eventhistory.EnableEventHistory;
import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import de.fraunhofer.iese.mydata.pip.EnablePolicyInformationPoint;
import de.fraunhofer.iese.mydata.pxp.EnablePolicyExecutionPoint;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
@EnablePolicyEnforcementPoint(basePackages = "de.thesis.consumer.backend.datasovereignty.pep")
@EnablePolicyInformationPoint
@EnablePolicyExecutionPoint
@EnableEventHistory
public class BackendApplication {

	public static void main(String[] args) {
		SpringApplication.run(BackendApplication.class, args);
	}
}
