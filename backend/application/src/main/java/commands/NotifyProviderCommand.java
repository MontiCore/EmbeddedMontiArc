package commands;

import lombok.Value;

@Value
public class NotifyProviderCommand {

	String url;
	String datasetId;
}
