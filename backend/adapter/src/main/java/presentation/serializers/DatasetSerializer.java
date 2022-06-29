package presentation.serializers;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;
import entity.Dataset;
import org.springframework.boot.jackson.JsonComponent;

import java.io.IOException;

@JsonComponent
public class DatasetSerializer extends StdSerializer<Dataset> {

	public DatasetSerializer() {
		this(null);
	}

	public DatasetSerializer(Class<Dataset> t) {
		super(t);
	}

	@Override
	public void serialize(
			Dataset dataset, JsonGenerator jgen, SerializerProvider provider)
			throws IOException {

		jgen.writeStartObject();
		jgen.writeStringField("id", dataset.getId().toString());
		jgen.writeStringField("offer", dataset.getOfferId().toString());
		jgen.writeObjectField("metadata", dataset.getMetadata());
		jgen.writeObjectField("data", dataset.getData());
		jgen.writeStringField("boughtAt", dataset.getBoughtAt().toString());
		jgen.writeEndObject();
	}
}
