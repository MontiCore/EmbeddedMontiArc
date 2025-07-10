package commands;

import lombok.Value;

import java.util.UUID;

@Value
public class BuyOfferCommand implements Command {
	UUID offerId;
}
