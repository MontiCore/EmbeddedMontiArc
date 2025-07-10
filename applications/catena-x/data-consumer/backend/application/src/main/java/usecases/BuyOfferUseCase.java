package usecases;

import commands.BuyOfferCommand;
import entity.Dataset;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.PolicyDeploymentPort;

import java.time.LocalDateTime;
import java.util.UUID;

@AllArgsConstructor
@Slf4j
public class BuyOfferUseCase implements CommandHandler<BuyOfferCommand> {

	private DatasetPersistencePort datasetPersistencePort;
	private OfferPersistencePort offerPersistencePort;
	private PolicyDeploymentPort policyDeploymentPort;

	@Override
	public void handle(BuyOfferCommand command) {
		Offer offer = offerPersistencePort.findById(command.getOfferId());
		Dataset dataset = createDatasetFromOffer(offer);
		dataset.setBoughtAt(LocalDateTime.now());
		dataset.getMetadata().getPolicy().setTargetId(dataset.getId());
		policyDeploymentPort.deployPolicy(dataset.getMetadata().getPolicy());
		datasetPersistencePort.save(dataset);
		offerPersistencePort.deleteById(offer.getId());

		log.info("Bought offer {}, created dataset {}", command.getOfferId().toString(), dataset.getId().toString());
	}

	private Dataset createDatasetFromOffer(Offer offer) {
		Dataset dataset = new Dataset();
		dataset.setId(UUID.randomUUID());
		dataset.setOfferId(offer.getId());
		dataset.setMetadata(offer.getMetadata());
		dataset.setData(offer.getData());

		return dataset;
	}
}
