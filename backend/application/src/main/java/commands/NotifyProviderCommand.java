package commands;

import lombok.Value;

import java.util.UUID;

@Value
public class NotifyProviderCommand implements Command {
	String url;
	UUID datasetId;
}
