package usecases;

import commands.AddOfferCommand;
import entity.Offer;
import lombok.AllArgsConstructor;
import ports.OfferPersistencePort;

import java.util.UUID;


@AllArgsConstructor
public class AddOfferUseCase implements CommandHandler<AddOfferCommand, Offer> {
	private final OfferPersistencePort offerPersistencePort;
	// TODO remove fixed id
	public Offer handle(AddOfferCommand command) {
		Offer offer = new Offer(
				UUID.fromString("ab1a041c-44a4-4af1-ace2-00af00ee3e49"),
				command.getMetadata(),
				command.getData()
		);

		offerPersistencePort.save(offer);

		return offer;
	}
}
