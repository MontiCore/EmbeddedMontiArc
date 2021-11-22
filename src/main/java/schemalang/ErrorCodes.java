package schemalang;

public final class ErrorCodes {

    private ErrorCodes() {
    }

    // Error codes for context conditions
    public static final String ERROR_CODE_SL_02C = "0xSL02";
    public static final String ERROR_MSG_SL_02C = " Parameter '%s' is already defined. A schema definition may only " +
            "contain unique members.";

    public static final String ERROR_CODE_SL_03C = "0xSL03";
    public static final String ERROR_MSG_SL_03C = " Parameter '%s' is already defined in schema '%s'.";

    public static final String ERROR_CODE_SL_04C = "0xSL04";
    public static final String ERROR_MSG_SL_04C = " Initial value %s is not compatible with type %s.";

    public static final String ERROR_CODE_SL_05C = "0xSL05";
    public static final String ERROR_MSG_SL_05C = " Initial value %s of property '%s' is not in domain %s.";

    public static final String ERROR_CODE_SL_06C = "0xSL06";
    public static final String ERROR_MSG_SL_06C = " Domain value '%s' of property '%s' is not compatible with type '%s'.";

    public static final String ERROR_CODE_SL_07C = "0xSL07";
    public static final String ERROR_MSG_SL_07C = " Unknown custom type '%s'.";

    public static final String ERROR_CODE_SL_08C = "0xSL08";
    public static final String ERROR_MSG_SL_08C = " The schema name and file name must be equal.";

    public static final String ERROR_CODE_SL_09C = "0xSL09";
    public static final String ERROR_MSG_SL_09C = " Parameter and object names are both named '%s', but they must be different.";

    public static final String ERROR_CODE_SL_17C = "0xSL17";
    public static final String ERROR_MSG_SL_17C = " Overriding the parameter '%s' is not allowed, because the schema " +
            "'%s' has no super schema.";

    public static final String ERROR_CODE_SL_18C = "0xSL18";
    public static final String ERROR_MSG_SL_18C = " Overriding the parameter '%s' is only allowed if this parameter is " +
            "already defined in a super schema.";

    public static final String ERROR_CODE_SL_19C = "0xSL19";
    public static final String ERROR_MSG_SL_19C = " Requires rule '%s' is already defined in schema '%s'.";

    public static final String ERROR_CODE_SL_20C = "0xSL20";
    public static final String ERROR_MSG_SL_20C = " Undefined parameter '%s' referenced in requires rule '%s'.";

    public static final String ERROR_CODE_SL_22C = "0xSL22";
    public static final String ERROR_MSG_SL_22C = " There must be at most one schema enum definitions.";

    public static final String ERROR_CODE_SL_23C = "0xSL23";
    public static final String ERROR_MSG_SL_23C = " The parameter '%s' has both a range and a domain. A range and a " +
            "domain may not be present at the same time.";

    public static final String ERROR_CODE_SL_24C = "0xSL24";
    public static final String ERROR_MSG_SL_24C = " Initial value %s of parameter '%s' is not in value range %s.";

    public static final String ERROR_CODE_TA_17C = "0xTA17C";
    public static final String ERROR_MSG_TA_17C = " Conflicting definition of property '%s'. The property is defined " +
            "in schema '%s', but also in reference model '%s'.";

    // Error codes for schema validation
    public static final String ERROR_CODE_SL_10C = "0xSL10C";
    public static final String ERROR_MSG_SL_10C = " Undefined parameter '%s'.";

    public static final String ERROR_CODE_SL_11C = "0xSL11C";
    public static final String ERROR_MSG_SL_11C = " Value of parameter '%s' is not compatible with type '%s'.";

    public static final String ERROR_CODE_SL_12C = "0xSL12C";
    public static final String ERROR_MSG_SL_12C = " Value '%s' is not applicable for parameter '%s'. The following " +
            "values are allowed: %s";

    public static final String ERROR_CODE_SL_13C = "0xSL13C";
    public static final String ERROR_MSG_SL_13C = " Undefined type '%s' for attribute '%s'.";

    public static final String ERROR_CODE_SL_14C = "0xSL14C";
    public static final String ERROR_MSG_SL_14C = " Value of parameter '%s' is not in value range '%s'.";

    public static final String ERROR_CODE_SL_15C = "0xSL15C";
    public static final String ERROR_MSG_SL_15C = " Value of parameter '%s' does not correspond to scale '%s'.";

    public static final String ERROR_CODE_SL_16C = "0xSL16C";
    public static final String ERROR_MSG_SL_16C = " Required parameter '%s' is missing.";

    public static final String ERROR_CODE_SL_21C = "0xSL21C";
    public static final String ERROR_MSG_SL_21C = " Setting parameter '%s' requires also setting parameter '%s'.";

    // Error codes for reference model validation
    public static final String ERROR_CODE_TA_00C = "0xTA00C";
    public static final String ERROR_MSG_TA_00C = " Validation based on reference model '%s' resulted in errors.";

    public static final String ERROR_CODE_TA_01C = "0xTA01C";
    public static final String ERROR_MSG_TA_01C = " No component with name '%s' is available.";

    public static final String ERROR_CODE_TA_02C = "0xTA02C";
    public static final String ERROR_MSG_TA_02C = " Component '%s' has the input ports '%s', but is only allowed to " +
            "have the ports '%s'.";

    public static final String ERROR_CODE_TA_03C = "0xTA03C";
    public static final String ERROR_MSG_TA_03C = " Component '%s' has the output ports '%s', but is only allowed to " +
            "have the ports '%s'.";

    public static final String ERROR_CODE_TA_04C = "0xTA04C";
    public static final String ERROR_MSG_TA_04C = " Component '%s' is expected to have an input port '%s'.";

    public static final String ERROR_CODE_TA_05C = "0xTA05C";
    public static final String ERROR_MSG_TA_05C = " Component '%s' is expected to have an output port '%s'.";

    public static final String ERROR_CODE_TA_06C = "0xTA06C";
    public static final String ERROR_MSG_TA_06C = " Port '%s' of component '%s' is expected to be %s-dimensional, but it is %s-dimensional.";

    public static final String ERROR_CODE_TA_07C = "0xTA07C";
    public static final String ERROR_MSG_TA_07C = " Port dimensions of components '%s' and '%s' do not match for connection '%s'.";

    public static final String ERROR_CODE_TA_08C = "0xTA08C";
    public static final String ERROR_MSG_TA_08C = " Port types of components '%s' and '%s' do not match for connection '%s'.";

    public static final String ERROR_CODE_TA_09C = "0xTA09C";
    public static final String ERROR_MSG_TA_09C = " Output port of connection '%s' has a port range defined, but the input port has none.";

    public static final String ERROR_CODE_TA_10C = "0xTA10C";
    public static final String ERROR_MSG_TA_10C = " Input port of connection '%s' has a port range defined, but the output port has none.";

    public static final String ERROR_CODE_TA_11C = "0xTA11C";
    public static final String ERROR_MSG_TA_11C = " Port ranges of components '%s' and '%s' do not match for connection '%s'.";

    public static final String ERROR_CODE_TA_13C = "0xTA13C";
    public static final String ERROR_MSG_TA_13C = " Port dimensions do not match for ports '%s'.";

    public static final String ERROR_CODE_TA_14C = "0xTA14C";
    public static final String ERROR_MSG_TA_14C = " Port types do not match for ports '%s'.";

    public static final String ERROR_CODE_TA_15C = "0xTA15C";
    public static final String ERROR_MSG_TA_15C = " Port ranges do not match for ports '%s'.";

    public static final String ERROR_CODE_TA_16C = "0xTA16C";
    public static final String ERROR_MSG_TA_16C = " No configuration entries for ports '%s' are defined.";

    public static final String ERROR_CODE_TA_18C = "0xTA18C";
    public static final String ERROR_MSG_TA_18C = " Port '%s' of component '%s' is expected to have a range %s.";

    public static final String ERROR_CODE_TA_19C = "0xTA19C";
    public static final String ERROR_MSG_TA_19C = " Configuration entry '%s' does not define configuration entries for " +
            "the ports '%s'.";

    public static final String ERROR_CODE_TA_20C = "0xTA20C";
    public static final String ERROR_MSG_TA_20C = " Port '%s' of component '%s' must be of primitive type %s, " +
            "but it is of type %s";

    public static final String ERROR_CODE_TA_21C = "0xTA21C";
    public static final String ERROR_MSG_TA_21C = " Port '%s' of component '%s' is expected to be %s-dimensional.";

    public static final String ERROR_CODE_TA_22C = "0xTA22C";
    public static final String ERROR_MSG_TA_22C = " Incompatible array port connection '%s' between components " +
            "'%s' and '%s'. Component '%s' has %s output ports, whereas component '%s' has %s input " +
            "ports.";

    public static final String ERROR_CODE_TA_23C = "0xTA23C";
    public static final String ERROR_MSG_TA_23C = " Incompatible array port connection '%s' between components " +
            "'%s' and '%s'. Component '%s' has the output port '%s', whereas component '%s' has the input " +
            "ports '%s'.";
}