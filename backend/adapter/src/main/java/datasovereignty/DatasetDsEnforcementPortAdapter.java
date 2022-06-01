package datasovereignty;


import de.fraunhofer.iese.mydata.policy.event.Event;
import entity.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Component;
import ports.DsEnforcementPort;

@Component
@AllArgsConstructor
public class DatasetDsEnforcementPortAdapter implements DsEnforcementPort<Dataset> {

	private final MydataDatasetPEP pep;

	@Override
	public Dataset enforce(Dataset dataset) {
		Event event = pep.enforce(dataset, dataset.getId().toString()).toBlocking().first();
		return event.getParameterValue("dataset", Dataset.class);
	}
}

