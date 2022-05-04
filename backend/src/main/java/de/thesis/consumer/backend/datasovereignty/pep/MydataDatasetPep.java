package de.thesis.consumer.backend.datasovereignty.pep;

import de.fraunhofer.iese.mydata.policy.event.Event;
import de.fraunhofer.iese.mydata.reactive.common.EventParameter;
import de.fraunhofer.iese.mydata.reactive.common.EventSpecification;
import de.fraunhofer.iese.mydata.reactive.common.Modifiers;
import de.fraunhofer.iese.mydata.reactive.common.PepServiceDescription;
import de.thesis.consumer.backend.domain.model.Dataset;
import rx.Observable;

@PepServiceDescription(componentName = "dataset-pep")
@Modifiers
public interface MydataDatasetPep {
	@EventSpecification(action = "dataset-access")
	Observable<Event> enforce(@EventParameter(name = "dataset") Dataset dataset, @EventParameter(name = "id") String id);
}
