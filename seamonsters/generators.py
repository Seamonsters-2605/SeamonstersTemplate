__author__ = "seamonsters"

import types
import itertools

class ParallelSignal:
    """
    A signal that can be returned from a generator in a parallel group to
    trigger an action on the group.
    """

class StopParallelSignal(ParallelSignal):
    """
    Value to signal that a group of parallel commands should be stopped.
    """

    def __init__(self, value=None):
        self.value = value

class AddParallelSignal(ParallelSignal):
    """
    Value to signal that a new generator should be added to the group of
    parallel commands.
    """

    def __init__(self, iterable):
        self.iterable = iterable


def sequence(*iterables):
    """
    Run a set of iterables sequentially
    """
    return itertools.chain(*iterables)

def parallel(*iterables):
    """
    Run a group of iterables in parallel. Ends when none are left running.

    An iterable can yield or return a ParallelSignal to trigger an action.
    """
    iterables = list(iterables)
    try:
        while len(iterables) != 0:
            toRemove = [ ]
            for iter in iterables:
                try:
                    result = next(iter)
                except StopIteration as e:
                    result = e.value
                    toRemove.append(iter)
                if isinstance(result, ParallelSignal):
                    if isinstance(result, StopParallelSignal):
                        return result.value
                    elif isinstance(result, AddParallelSignal):
                        iterables.append(result.iterable)
            for iter in toRemove:
                iterables.remove(iter)
            yield
    finally:
        for iter in iterables:
            iter.close()

def wait(time):
    """
    Wait for a certain number of iterations.
    """
    for _ in range(time):
        yield

def forever():
    """
    Iterate forever.
    """
    while True:
        yield

def timeLimit(iterable, time):
    """
    Run the iterable until it finishes or the given time limit has passed.
    Return the value of the iterable if it ends early, None otherwise.
    """
    return itertools.islice(iterable, time)

def untilTrue(iterable):
    """
    Run the iterable until it yields True, then stop.
    """
    return itertools.takewhile(lambda x: not x, iterable)

def ensureTrue(iterable, requiredCount):
    """
    Wait until the iterable yields True for a certain number of consecutive
    iterations before finishing.
    """
    count = 0
    for x in iterable:
        if x:
            count += 1
        else:
            count = 0
        if count > requiredCount:
            break
        yield

def returnValue(iterable, value):
    """
    Run an iterable but change the return value.

    :return: value
    """
    yield from iterable
    return value

def stopAllWhenDone(iterable, value=None):
    """
    If run in a ``sea.parallel`` block, when the iterable completes all
    parallel commands will be stopped.
    """
    return returnValue(iterable, StopParallelSignal(value))
