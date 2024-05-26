import time
import threading

class MyClass1:
    def __init__(self, parent):
        self.parent = parent

    def say_hello(self):
        while True:
            print(f"Hello from {self.parent.name}!")
            time.sleep(3)

class MyClass2:
    def __init__(self, parent):
        self.parent = parent
        self.name_instance = MyClass1(self.parent)

    def start_threads(self):
        hello_thread = threading.Thread(target=self.name_instance.say_hello)
        hello_thread.start()
        while True:
            print(f"Displayed name: {self.parent.name}")
            time.sleep(3)

class Controller:
    def __init__(self, age):
        self.age = age
        self._name = "John Doe"

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value

    def person(self):
        name_instance2 = MyClass2(self)
        display_thread = threading.Thread(target=name_instance2.start_threads)
        display_thread.start()
        while True:
            print(f"I am {self.age} years old.")
            time.sleep(3)
            self.name = "Son"

# Tạo đối tượng của Controller và bắt đầu
test = Controller(25)
test.person()
