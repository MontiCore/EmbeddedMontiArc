package commands;

import entity.DataRow;
import entity.Metadata;
import lombok.AccessLevel;
import lombok.AllArgsConstructor;
import lombok.NoArgsConstructor;
import lombok.Value;

import java.util.List;

@Value
@NoArgsConstructor(force = true, access = AccessLevel.PRIVATE)
@AllArgsConstructor
public class AddOfferCommand implements Command {
	Metadata metadata;
	List<DataRow> data;
}
