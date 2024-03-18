class test():
    def __init__(self):
        pass
    def basic_func(self):
        def sum(self,a,b):
            return a + b
    def result(self):
        return self.sum(5,10)
testnum = test()
testnum.basic_func.sum(5, 10)
print("the sum is: %d")