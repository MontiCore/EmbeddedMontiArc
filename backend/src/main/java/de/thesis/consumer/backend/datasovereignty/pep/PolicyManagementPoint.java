package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import de.fraunhofer.iese.mydata.exception.ConflictingResourceException;
import de.fraunhofer.iese.mydata.exception.InvalidEntityException;
import de.fraunhofer.iese.mydata.exception.NoSuchEntityException;
import de.fraunhofer.iese.mydata.exception.ResourceUpdateException;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.PolicyInstantiationException;
import de.thesis.consumer.backend.domain.model.Policy;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;

import java.io.IOException;

@Component
@AllArgsConstructor
public class PolicyManagementPoint implements IPolicyManagementPoint {

	private final IMyDataEnvironment myDataEnv;

	@Override
	public void instantiatePolicy(Policy policy) throws PolicyInstantiationException {
		try {
			System.err.println(policy.getRawValue());
			myDataEnv.getPmp().deployPolicy(myDataEnv.getPmp().addPolicy(new de.fraunhofer.iese.mydata.policy.Policy(
					"<policy id='urn:policy:rwth-student-solution:0c2233b4-911e-451b-96e7-0ca13088e739' xmlns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:tns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:parameter='http://www.mydata-control.de/4.0/parameter' xmlns:pip='http://www.mydata-control.de/4.0/pip' xmlns:function='http://www.mydata-control.de/4.0/function' xmlns:event='http://www.mydata-control.de/4.0/event' xmlns:constant='http://www.mydata-control.de/4.0/constant' xmlns:variable='http://www.mydata-control.de/4.0/variable' xmlns:variableDeclaration='http://www.mydata-control.de/4.0/variableDeclaration' xmlns:valueChanged='http://www.mydata-control.de/4.0/valueChanged' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xmlns:date='http://www.mydata-control.de/4.0/date' xmlns:time='http://www.mydata-control.de/4.0/time' xmlns:day='http://www.mydata-control.de/4.0/day'>\n" +
							"    <mechanism event='urn:action:rwth-student-solution:dataset-access'>\n" +
							"        <if>\n" +
							"            <equals>\n" +
							"        			 <constant:string value='0c2233b4-911e-451b-96e7-0ca13088e739'/>\n" +
							"        		  	 <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>\n" +
							"      		 </equals>\n" +
							"            <then>\n" +
							"                <modify eventParameter='dataset' method='replace' jsonPathQuery='$.description'>\n" +
							"                    <parameter:string name='replaceWith' value='testwert'/>\n" +
							"                </modify>\n" +
							"            </then>\n" +
							"        </if>\n" +
							"    </mechanism>\n" +
							"</policy>")));
		} catch (IOException | NoSuchEntityException | ResourceUpdateException | InvalidEntityException | ConflictingResourceException e) {
			throw new PolicyInstantiationException(String.format("Error instantiating policy with ID %s", policy.getId()));
		}
	}
}
