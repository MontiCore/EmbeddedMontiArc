package usecases;

import commands.AddOfferCommand;
import entity.Metadata;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;

import java.time.LocalDate;
import java.util.UUID;


@AllArgsConstructor
public class AddOfferUseCase implements CommandHandler<AddOfferCommand, Offer> {

	// private final DataRowPersistencePort dataRowPersistencePort;
	private final OfferPersistencePort offerPersistencePort;

	public Offer handle(AddOfferCommand command) {
		Metadata metadata = new Metadata();
		metadata.setTitle(command.getTitle());
		metadata.setProvider(command.getProvider());
		metadata.setDescription(command.getDescription());
		metadata.setPrice(command.getPrice());
		metadata.setExpiresOn(command.getExpiresOn());
		metadata.setLoggingUrl(command.getLoggingUrl());
		metadata.setPolicy(command.getPolicy());

		Offer offer = new Offer(
				UUID.randomUUID(),
				metadata,
				command.getData()
		);

		offerPersistencePort.save(offer);

		return offer;
	}
}
