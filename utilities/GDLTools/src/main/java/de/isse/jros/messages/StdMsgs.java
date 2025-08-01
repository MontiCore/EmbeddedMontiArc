/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. 
 *
 * Copyright 2016-2019 ISSE, University of Augsburg 
 */

package de.isse.jros.messages;

import de.isse.jros.fields.RInteger;
import de.isse.jros.fields.RMessage;
import de.isse.jros.fields.RBoolean;
import de.isse.jros.fields.RString;
import de.isse.jros.fields.RTime;
import de.isse.jros.fields.RFloat;

import de.isse.jros.fields.RArray;

import de.isse.jros.types.ROSstruct;
import de.isse.jros.types.ROSarray;

import de.isse.jros.types.ROSfloat32;


/** Standard ROS Messages */
public class StdMsgs {

	/** Standard metadata for higher-level stamped data types */
	public static ROSstruct Header() {
		return new ROSstruct("std_msgs/Header").withUint32("seq").withTime("stamp").withString("frame_id");
	}

	/** Standard metadata for higher-level stamped data types */
	public static class Header extends RMessage {
		public final RInteger seq = field("seq", RInteger.uint32());
		public final RTime stamp = field("stamp", RTime.time());
		public final RString frame_id = field("frame_id", RString.string());
		public final ROSstruct TYPE = type("std_msgs/Header");
	}

	public static ROSstruct MultiArrayDimension() {
		return new ROSstruct("std_msgs/MultiArrayDimension").withString("label").withUint32("size").withUint32("stride");
	}

	/* Standard Messages */

	public static class MultiArrayDimension extends RMessage {
		public final RString label = field("label", RString.string());
		public final RInteger size = field("size", RInteger.uint32());
		public final RInteger stride = field("stride", RInteger.uint32());
		public final ROSstruct TYPE = type("std_msgs/MultiArrayDimension");
	}

	public static ROSstruct MultiArrayLayout() {
		return new ROSstruct("std_msgs/MultiArrayLayout")
			.withField("dim", new ROSarray(new MultiArrayDimension().TYPE))
			.withUint32("data_offset");
	}

	public static class MultiArrayLayout extends RMessage {
		public final RArray<MultiArrayDimension> dim = array("dim", new MultiArrayDimension());
		public final RInteger data_offset = field("data_offset", RInteger.uint32());
		public final ROSstruct TYPE = type("std_msgs/MultiArrayLayout");
	}

	public static ROSstruct Float32MultiArray() {
		return new ROSstruct("std_msgs/Float32MultiArray")
			.withField("layout", MultiArrayLayout())
			.withField("data", new ROSarray(ROSfloat32.TYPE));
	}

	public static class Float32MultiArray extends RMessage {
		public final MultiArrayLayout layout = field("layout", new MultiArrayLayout());
		public final RArray<RFloat> data = array("data", RFloat.float32());
		public final ROSstruct TYPE = type("std_msgs/Float32MultiArray");
	}
	
	public static ROSstruct Int32() {
		return new ROSstruct("std_msgs/Int32").withInt32("data");
	}

	public static class Int32 extends RMessage {
		public final RInteger data = field("data", RInteger.int32());
		public final ROSstruct TYPE = type("std_msgs/Int32");
	}

	public static ROSstruct Float32() {
		return new ROSstruct("std_msgs/Float32").withFloat32("data");
	}

	public static class Float32 extends RMessage {
		public final RFloat data = field("data", RFloat.float32());
		public final ROSstruct TYPE = type("std_msgs/Float32");
	}

	public static ROSstruct Bool() {
		return new ROSstruct("std_msgs/Bool").withBool("bool");
	}

	public static class Bool extends RMessage {
		public final RBoolean data = field("bool", RBoolean.bool());
		public final ROSstruct TYPE = type("std_msgs/Bool");
	}

}
