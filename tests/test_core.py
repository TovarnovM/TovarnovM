# import sys
# import os
# wd = os.path.abspath(__file__) # os.path.dirname(os.path.dirname(os.getcwd()))
# wd = os.path.dirname(wd)
# sys.path.append(wd)

# from core import DObject, DParam, dobefore, doafter

# class SomeTst(DObject):
#     f1 = DParam()
#     f2 = DParam()

#     @dobefore
#     def foo_before(self):
#         pass

#     @doafter
#     def foo_after(self):
#         pass

# o1 = SomeTst()
# o2 = SomeTst()

# o1.add_child(o2)

for i in range(3,5):
    print(i)