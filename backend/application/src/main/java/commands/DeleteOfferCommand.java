package commands;

import lombok.Value;

import java.util.UUID;

@Value
public class DeleteOfferCommand implements Command {
	UUID offerId;
}
