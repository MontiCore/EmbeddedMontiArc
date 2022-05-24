package presentation.controller;

import dto.AddOfferCommand;
import exception.PolicyInvalidException;
import lombok.AllArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.*;
import usecase.AddOfferUseCase;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin(origins = "*")
@Slf4j
public class OfferController {

	private AddOfferUseCase addOfferUseCase;

	@PostMapping(consumes = MediaType.APPLICATION_JSON_VALUE)
	@ResponseStatus(HttpStatus.OK)
	public void offer(@RequestBody AddOfferCommand offer) {
		try {
			addOfferUseCase.addOffer(offer);
		} catch (PolicyInvalidException e) {
			System.err.println("Policy not valid");
		}
	}
}
