package de.thesis.consumer;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.fraunhofer.iese.mydata.eventhistory.EnableEventHistory;
import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import de.fraunhofer.iese.mydata.policy.Policy;
import de.fraunhofer.iese.mydata.pxp.EnablePolicyExecutionPoint;
import de.fraunhofer.iese.mydata.timer.Timer;
import lombok.AllArgsConstructor;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.context.annotation.ComponentScan;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import utilities.MydataResourceReader;

import javax.annotation.PostConstruct;

@SpringBootApplication
@ComponentScan(basePackages = {"presentation", "persistence", "datasovereignty", "insurance", "webclient", "de.thesis.consumer.config"})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan(basePackages = "persistence.entity")
@EnablePolicyEnforcementPoint(basePackages = "datasovereignty")
@EnablePolicyExecutionPoint
@EnableEventHistory
@AllArgsConstructor
public class ConsumerSpringBootApplication {

	private IMyDataEnvironment myDataEnvironment;

	public static void main(String[] args) {
		SpringApplication.run(ConsumerSpringBootApplication.class, args);
	}

	@PostConstruct
	public void setUp() {
		Policy policy = MydataResourceReader.readPolicyFromFile("expiration_check_policy.xml");
		Timer timer = MydataResourceReader.readTimerFromFile("expiration_check_timer.xml");

		try {
			myDataEnvironment.getPmp().deployPolicy(myDataEnvironment.getPmp().addPolicy(policy));
			myDataEnvironment.getPmp().deployTimer(myDataEnvironment.getPmp().addTimer(timer));
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
