__author__ = "seamonsters"

import types

def _changeModule(module):
    for itemName in dir(module):
        item = module.__dict__.get(itemName)
        if isinstance(item, types.FunctionType) or isinstance(item, type):
            if(item.__module__.startswith("seamonsters")):
                item.__module__ = "seamonsters"

from .bot import *
from .dashboard import *
from .drive import *
from .gamepad import *
from .generators import *
from .joystick import *
from .logging import *
from .path import *
from .superHolonomicDrive import *

_changeModule(bot)
_changeModule(dashboard)
_changeModule(drive)
_changeModule(gamepad)
_changeModule(generators)
_changeModule(joystick)
_changeModule(logging)
_changeModule(path)
_changeModule(superHolonomicDrive)
