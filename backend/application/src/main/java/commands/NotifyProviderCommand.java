package commands;

import lombok.Value;

@Value
public class NotifyProviderCommand implements Command {
	String url;
	String datasetId;
}
