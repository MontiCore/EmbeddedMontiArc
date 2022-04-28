package de.thesis.consumer.backend;

import de.fraunhofer.iese.mydata.eventhistory.EnableEventHistory;
import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import de.fraunhofer.iese.mydata.pip.EnablePolicyInformationPoint;
import de.fraunhofer.iese.mydata.pxp.EnablePolicyExecutionPoint;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.model.Timer;
import lombok.AllArgsConstructor;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

import javax.annotation.PostConstruct;

@SpringBootApplication
@EnablePolicyEnforcementPoint(basePackages = "de.thesis.consumer.backend.datasovereignty.pep")
@EnablePolicyInformationPoint
@EnablePolicyExecutionPoint
@EnableEventHistory
@AllArgsConstructor
public class BackendApplication {

	private IPolicyManagementPoint pmp;

	public static void main(String[] args) {
		SpringApplication.run(BackendApplication.class, args);
	}

	@PostConstruct
	public void setUp() {
		Policy policy = MydataResourceReader.readPolicyFromFile("expiration_check_policy.xml");
		pmp.deployPolicy(policy);

		Timer timer = MydataResourceReader.readTimerFromFile("expiration_check_timer.xml");
		pmp.deployTimer(timer);
	}
}

