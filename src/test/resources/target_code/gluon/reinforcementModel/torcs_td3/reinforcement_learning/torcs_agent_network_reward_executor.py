# (c) https://github.com/MontiCore/monticore  
# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_torcs_agent_network_reward_executor', [dirname(__file__)])
        except ImportError:
            import _torcs_agent_network_reward_executor
            return _torcs_agent_network_reward_executor
        if fp is not None:
            try:
                _mod = imp.load_module('_torcs_agent_network_reward_executor', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _torcs_agent_network_reward_executor = swig_import_helper()
    del swig_import_helper
else:
    import _torcs_agent_network_reward_executor
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0


class torcs_agent_network_reward_input(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, torcs_agent_network_reward_input, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, torcs_agent_network_reward_input, name)
    __repr__ = _swig_repr
    __swig_setmethods__["state"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_state_set
    __swig_getmethods__["state"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_state_get
    if _newclass:
        state = _swig_property(_torcs_agent_network_reward_executor.torcs_agent_network_reward_input_state_get, _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_state_set)
    __swig_setmethods__["isTerminal"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_isTerminal_set
    __swig_getmethods__["isTerminal"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_isTerminal_get
    if _newclass:
        isTerminal = _swig_property(_torcs_agent_network_reward_executor.torcs_agent_network_reward_input_isTerminal_get, _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_isTerminal_set)

    def __init__(self):
        this = _torcs_agent_network_reward_executor.new_torcs_agent_network_reward_input()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _torcs_agent_network_reward_executor.delete_torcs_agent_network_reward_input
    __del__ = lambda self: None
torcs_agent_network_reward_input_swigregister = _torcs_agent_network_reward_executor.torcs_agent_network_reward_input_swigregister
torcs_agent_network_reward_input_swigregister(torcs_agent_network_reward_input)

class torcs_agent_network_reward_output(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, torcs_agent_network_reward_output, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, torcs_agent_network_reward_output, name)
    __repr__ = _swig_repr
    __swig_setmethods__["reward"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_output_reward_set
    __swig_getmethods__["reward"] = _torcs_agent_network_reward_executor.torcs_agent_network_reward_output_reward_get
    if _newclass:
        reward = _swig_property(_torcs_agent_network_reward_executor.torcs_agent_network_reward_output_reward_get, _torcs_agent_network_reward_executor.torcs_agent_network_reward_output_reward_set)

    def __init__(self):
        this = _torcs_agent_network_reward_executor.new_torcs_agent_network_reward_output()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _torcs_agent_network_reward_executor.delete_torcs_agent_network_reward_output
    __del__ = lambda self: None
torcs_agent_network_reward_output_swigregister = _torcs_agent_network_reward_executor.torcs_agent_network_reward_output_swigregister
torcs_agent_network_reward_output_swigregister(torcs_agent_network_reward_output)

class torcs_agent_network_reward_executor(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, torcs_agent_network_reward_executor, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, torcs_agent_network_reward_executor, name)
    __repr__ = _swig_repr

    def init(self):
        return _torcs_agent_network_reward_executor.torcs_agent_network_reward_executor_init(self)

    def execute(self, input):
        return _torcs_agent_network_reward_executor.torcs_agent_network_reward_executor_execute(self, input)

    def __init__(self):
        this = _torcs_agent_network_reward_executor.new_torcs_agent_network_reward_executor()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _torcs_agent_network_reward_executor.delete_torcs_agent_network_reward_executor
    __del__ = lambda self: None
torcs_agent_network_reward_executor_swigregister = _torcs_agent_network_reward_executor.torcs_agent_network_reward_executor_swigregister
torcs_agent_network_reward_executor_swigregister(torcs_agent_network_reward_executor)

# This file is compatible with both classic and new-style classes.


