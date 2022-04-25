package de.thesis.consumer.backend;

import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
@EnablePolicyEnforcementPoint(basePackages = "de.thesis.consumer.backend.datasovereignty.pep")
public class BackendApplication {

	private static final Logger LOG = LoggerFactory.getLogger(BackendApplication.class);

	public static void main(String[] args) {
		SpringApplication.run(BackendApplication.class, args);
	}
}
