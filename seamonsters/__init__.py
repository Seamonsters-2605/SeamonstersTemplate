__author__ = "seamonsters"

import types

def _changeModule(module):
    for itemName in dir(module):
        item = module.__dict__.get(itemName)
        if isinstance(item, types.FunctionType) or isinstance(item, type):
            if(item.__module__.startswith("seamonsters")):
                item.__module__ = "seamonsters"

from .bot import *
_changeModule(bot)

try:
    from .dashboard import *
    _changeModule(dashboard)
except ModuleNotFoundError:
    pass

from .drive import *
_changeModule(drive)

from .gamepad import *
_changeModule(gamepad)

from .generators import *
_changeModule(generators)

from .joystick import *
_changeModule(joystick)

from .path import *
_changeModule(path)

from .superHolonomicDrive import *
_changeModule(superHolonomicDrive)
