package datasovereignty;

import de.fraunhofer.iese.mydata.policy.event.Event;
import de.fraunhofer.iese.mydata.reactive.common.EventParameter;
import de.fraunhofer.iese.mydata.reactive.common.EventSpecification;
import de.fraunhofer.iese.mydata.reactive.common.PepServiceDescription;
import entity.Dataset;
import rx.Observable;

@PepServiceDescription(componentName = "dataset-pep")
public interface MydataDatasetPEP {
	@EventSpecification(action = "dataset-access")
	Observable<Event> enforce(@EventParameter(name = "dataset") Dataset dataset, @EventParameter(name = "id") String id);
}
