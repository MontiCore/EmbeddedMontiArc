package presentation.controllers;

import commands.AddOfferCommand;
import commands.BuyOfferCommand;
import entity.Dataset;
import entity.Metadata;
import entity.Offer;
import exceptions.UseCaseException;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.server.ResponseStatusException;
import queries.GetAllOffersMetadataQuery;
import usecases.AddOfferUseCase;
import usecases.BuyOfferUseCase;
import usecases.GetAllOffersMetadataUseCase;

import java.util.Map;
import java.util.UUID;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
@Slf4j
public class OfferController {

	private AddOfferUseCase addOfferUseCase;
	private BuyOfferUseCase buyOfferUseCase;

	private GetAllOffersMetadataUseCase getAllOffersUseCase;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void addOffer(@RequestBody AddOfferCommand addOfferCommand) {
		try {
			Offer offer = addOfferUseCase.handle(addOfferCommand);

			log.info("Created offer {}", offer.getId());
		} catch (UseCaseException exception) {
			log.error("Error while creating offer");
			throw new ResponseStatusException(HttpStatus.BAD_REQUEST, "Dataset could not be offered", exception);
		}
	}

	@PostMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void buyOffer(@PathVariable UUID offerId) {
		Dataset dataset = buyOfferUseCase.handle(new BuyOfferCommand(offerId));

		log.info("Bought offer {}, created dataset {}", offerId, dataset.getId());
	}

	@GetMapping
	public Map<UUID, Metadata> getAllOffers() {
		return getAllOffersUseCase.handle(new GetAllOffersMetadataQuery());
	}
}
