package usecases;

import commands.BuyOfferCommand;
import entity.Dataset;
import entity.Offer;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import ports.DatasetPersistencePort;
import ports.DsManagementPort;
import ports.OfferPersistencePort;

import java.time.LocalDateTime;
import java.util.UUID;

@AllArgsConstructor
@Slf4j
public class BuyOfferUseCase implements CommandHandler<BuyOfferCommand> {

	private DatasetPersistencePort datasetPersistencePort;
	private OfferPersistencePort offerPersistencePort;
	private DsManagementPort dsManagementPort;

	@Override
	public void handle(BuyOfferCommand command) {
		Offer offer = offerPersistencePort.findBy(command.getOfferId());
		Dataset dataset = createDatasetFromOffer(offer);
		dataset.setBoughtAt(LocalDateTime.now());
		dataset.getMetadata().getPolicy().setTargetId(dataset.getId());
		dsManagementPort.deployPolicy(dataset.getMetadata().getPolicy());
		datasetPersistencePort.save(dataset);

		log.info("Bought offer {}, created dataset {}", command.getOfferId().toString(), dataset.getId().toString());
	}

	private Dataset createDatasetFromOffer(Offer offer) {
		Dataset dataset = new Dataset();
		dataset.setId(UUID.randomUUID());
		dataset.setOffer(offer);
		dataset.setMetadata(offer.getMetadata());
		dataset.setData(offer.getData());

		return dataset;
	}
}
