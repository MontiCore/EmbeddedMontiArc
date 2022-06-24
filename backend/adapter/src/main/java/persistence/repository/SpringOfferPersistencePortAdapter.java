package persistence.repository;

import entity.Offer;
import lombok.AllArgsConstructor;
import persistence.entity.OfferEntity;
import persistence.mappers.Mapper;
import ports.OfferPersistencePort;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

@AllArgsConstructor
public class SpringOfferPersistencePortAdapter implements OfferPersistencePort {

	private final Mapper<Offer, OfferEntity> mapper;
	private final SpringOfferRepository repository;

	@Override
	public void save(Offer offer) {
		OfferEntity offerEntity = mapper.mapTo(offer);
		offerEntity.getData().forEach(dataRowEntity ->
				dataRowEntity.setOffer(offerEntity)
		);

		repository.save(offerEntity);
	}

	@Override
	public Offer findById(UUID id) {
		Optional<OfferEntity> optional = repository.findById(id);

		if (optional.isEmpty()) {
			return null;
		}

		return mapper.mapFrom(optional.get());
	}

	@Override
	public Iterable<Offer> findAll() {
		List<Offer> offers = new ArrayList<>();
		repository.findAll().forEach(offerEntity ->
				offers.add(mapper.mapFrom(offerEntity))
		);

		return offers;
	}

	@Override
	public void deleteById(UUID offerId) {
		repository.deleteByIdWithoutCascading(offerId);
	}

	@Override
	public void deleteDatasetDataRowOfferById(UUID offerId) {
		repository.deleteById(offerId);
	}
}
