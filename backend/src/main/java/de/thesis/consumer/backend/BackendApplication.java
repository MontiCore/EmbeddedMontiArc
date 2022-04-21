package de.thesis.consumer.backend;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.fraunhofer.iese.mydata.pep.EnablePolicyEnforcementPoint;
import de.fraunhofer.iese.mydata.policy.Policy;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

import javax.annotation.PostConstruct;
import java.io.IOException;

@SpringBootApplication
@EnablePolicyEnforcementPoint(basePackages = "de.thesis.consumer.backend.datasovereignty.pep")
public class BackendApplication {

	@Autowired
	private IMyDataEnvironment myDataEnvironment;

	private static final Logger LOG = LoggerFactory.getLogger(BackendApplication.class);
	public static void main(String[] args) {
		SpringApplication.run(BackendApplication.class, args);
	}

	@PostConstruct
	public void init() {
		LOG.info("INIT:begin");
		try {
			// TODO this is a hardcoded policy for demonstration purpose
			myDataEnvironment.getPmp().deployPolicy(myDataEnvironment.getPmp().addPolicy(new Policy(
					"<policy id='urn:policy:rwth-student-solution:1234' xmlns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:tns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:parameter='http://www.mydata-control.de/4.0/parameter' xmlns:pip='http://www.mydata-control.de/4.0/pip' xmlns:function='http://www.mydata-control.de/4.0/function' xmlns:event='http://www.mydata-control.de/4.0/event' xmlns:constant='http://www.mydata-control.de/4.0/constant' xmlns:variable='http://www.mydata-control.de/4.0/variable' xmlns:variableDeclaration='http://www.mydata-control.de/4.0/variableDeclaration' xmlns:valueChanged='http://www.mydata-control.de/4.0/valueChanged' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xmlns:date='http://www.mydata-control.de/4.0/date' xmlns:time='http://www.mydata-control.de/4.0/time' xmlns:day='http://www.mydata-control.de/4.0/day'>\n" +
							"    <mechanism event='urn:action:rwth-student-solution:dataset-access'>\n" +
							"        <if>\n" +
							"            <time is='after' value='14:00'/>\n" +
							"            <then>\n" +
							"                <modify eventParameter='dataset' method='replace' jsonPathQuery='$.description'>\n" +
							"                    <parameter:string name='replaceWith' value='testwert'/>\n" +
							"                </modify>\n" +
							"            </then>\n" +
							"        </if>\n" +
							"    </mechanism>\n" +
							"</policy>")));
		} catch (IOException | ResourceUpdateException | InvalidEntityException | ConflictingResourceException | NoSuchEntityException e) {
			LOG.error(e.getMessage(), e);
		}
		LOG.info("INIT:end");
	}
}
