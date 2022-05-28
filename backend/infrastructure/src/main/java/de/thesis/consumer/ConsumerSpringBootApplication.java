package de.thesis.consumer;

import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;

@SpringBootApplication
@ComponentScan(basePackages = {"presentation", "persistence", "datasovereignty", "de.thesis.consumer.config"})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan(basePackages = "persistence.entity")
@EnablePolicyEnforcementPoint(basePackages = "datasovereignty")
public class ConsumerSpringBootApplication {

	public static void main(String[] args) {
		SpringApplication.run(ConsumerSpringBootApplication.class, args);
	}

}
