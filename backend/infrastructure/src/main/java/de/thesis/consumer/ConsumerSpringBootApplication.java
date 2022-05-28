package de.thesis.consumer;

import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import lombok.AllArgsConstructor;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import ports.DatasetDeletionExecutionPoint;
import ports.PolicyManagementPort;

import javax.annotation.PostConstruct;

@SpringBootApplication
@ComponentScan(basePackages = {"presentation", "persistence", "datasovereignty", "webclient", "de.thesis.consumer.config"})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan(basePackages = "persistence.entity")
@EnablePolicyEnforcementPoint(basePackages = "datasovereignty")
@AllArgsConstructor
public class ConsumerSpringBootApplication {

	private PolicyManagementPort policyManagementPort;
	private DatasetDeletionExecutionPoint datasetDeletionExecutionPoint;

	public static void main(String[] args) {
		SpringApplication.run(ConsumerSpringBootApplication.class, args);
	}

	@PostConstruct
	public void init() {
		policyManagementPort.addPxp(datasetDeletionExecutionPoint);
	}
}
