package presentation.controllers;

import commands.AddOfferCommand;
import commands.BuyOfferCommand;
import commands.DeleteOfferCommand;
import entity.Metadata;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import queries.GetOffersMetadataQuery;
import usecases.AddOfferUseCase;
import usecases.BuyOfferUseCase;
import usecases.DeleteOfferUseCase;
import usecases.GetOffersMetadataUseCase;

import java.util.Map;
import java.util.UUID;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class OfferController {

	private AddOfferUseCase addOfferUseCase;
	private BuyOfferUseCase buyOfferUseCase;
	private GetOffersMetadataUseCase getAllOffersUseCase;
	private DeleteOfferUseCase deleteOfferUseCase;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void addOffer(@RequestBody AddOfferCommand addOfferCommand) {
		addOfferUseCase.handle(addOfferCommand);
	}

	@PostMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void buyOffer(@PathVariable UUID offerId) {
		buyOfferUseCase.handle(new BuyOfferCommand(offerId));
	}

	@GetMapping
	public Map<UUID, Metadata> getAllOffers(@RequestParam(required = false) boolean bought) {
		return getAllOffersUseCase.handle(new GetOffersMetadataQuery(bought));
	}

	@DeleteMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void deleteOffer(@PathVariable UUID offerId) {
		deleteOfferUseCase.handle(new DeleteOfferCommand(offerId));
	}
}
