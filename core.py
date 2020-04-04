import numpy as np

class DParam:
    def __init__(self, value=0.0, name=None, parent=None):
        self.name = '' if name is None else name
        self.value = value
        self.parent = parent
        self.ind = -1
        self.diff = None
        self.enabled = True

    def set_ind(self, i0):
        self.ind = i0
        return i0 + 1

    def get_value(self, t):
        return self.value

    def set_value(self, value):
        self.value = value

    def synch(self, t, y):
        if self.ind >=0 and self.enabled:
            self.set_value(y[self.ind])
    
    @property
    def full_name(self):
        if self.parent is not None:
            s = self.parent.full_name
            return s + '.' + self.name
        return self.name

    def fill_dy(self, t, dy):
        if self.ind >=0 and (self.diff is not None):
            if self.enabled:
                dy[self.ind] = self.diff.get_value(t)
            else:
                dy[self.ind] = 0
    
    def fill_y(self, t, y):
        if self.ind >= 0:
            y[self.ind] = self.get_value(t)


class DObject:
    def __init__(self, name=None, parent=None):
        self.parent = parent
        self.name = '' if name is None else name
        self._dparams_my = []
        self.diffs = []
        self._dparams_all = []
        self.children = []
        self.__dobefore_foos = set()
        self.__doafter_foos = set()
        self.__init_attrs()
        self.__init_beforeafters_foos()
        self._enabled = True

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, value):
        self._enabled = value
        for dp in self._dparams_my:
            dp.enabled = value

    @property
    def full_name(self):
        if self.parent is not None:
            s = self.parent.full_name
            return s + '.' + self.name
        return self.name

    def get_diffs(self):
        for dp in self._dparams_my:
            if dp.diff:
                yield dp
        for child in self.children:
            yield from child.get_diffs()

    def get_all_dparams(self):
        for dp in self._dparams_my:
            yield dp
        for child in self.children:
            yield from child.get_all_dparams()

    def rebuild(self):
        self._dparams_all = list(self.get_all_dparams())
        for dp in self._dparams_all:
            dp.ind = -1
        self.diffs = list(self.get_diffs())     
        i = 0   
        for dp in self.diffs:
            i = dp.set_ind(i)

    def get_y0(self, t0=0, rebuild=True):
        if rebuild:
            self.rebuild()
        n = len(self.diffs)
        res = np.zeros(n)
        for dp in self.diffs:
            dp.fill_y(t0, res)
        return res

    def synch(self, t, y):
        self._dobefore(t, y)
        for dp in self._dparams_all():
            if dp.enabled:
                dp.synch(t, y)
        self._doafter(t, y)         

    def get_dydt(self, t, y):
        self.synch(t, y)
        res = np.zeros_like(y)
        for dp in self.diffs:
            dp.fill_dy(t, res)
        return res

    def __init_attrs(self):
        for attr_name in self.__dir__():
            if (not attr_name.startswith('__')):
                attr = getattr(self, attr_name)
                if isinstance(attr, DParam):
                    my_dparam = DParam(attr._value, attr_name)
                    setattr(self, attr_name, my_dparam)
                    self.__add_dparam(my_dparam)

    def __init_beforeafters_foos(self):
        for attr_name in self.__dir__():
            attr = getattr(self, attr_name)
            if hasattr(attr, 'dobefore_flag') and attr.dobefore_flag:
                self.__dobefore_foos.add(attr)
            if hasattr(attr, 'doafter_flag') and attr.doafter_flag:
                self.__doafter_foos.add(attr)

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

    def __add_dparam(self, dparam: DParam):
        self._dparams_my.append(dparam)
        dparam.parent = self

    def _dobefore(self, t, y):
        for foo in self.__dobefore_foos:
            foo(t, y)
        for child in self.children:
            child._dobefore(t, y)

    def _doafter(self, t, y):
        for foo in self.__doafter_foos:
            foo(t, y)
        for child in self.children:
            child._doafter(t, y)


def dobefore(method):
    method.dobefore_flag = True
    return method

def doafter(method):
    method.doafter_flag = True
    return method



class SomeTst(DObject):
    f1 = DParam()
    f2 = DParam(1)

    def __init__(self, name=None, parent=None):
        super().__init__(name=name, parent=parent)
        self.f1.diff = DParam(10)

    @dobefore
    def foo2(self, t, y):
        print('foo2')

    @doafter
    def foo(self, t, y):
        print('foo')



d = SomeTst('d')
d2 = SomeTst('d2')
d3 = SomeTst('d3')
d.add_child(d2)
d2.add_child(d3)

for dp in d.get_diffs():
    print(dp.full_name)

