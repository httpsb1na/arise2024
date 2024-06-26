class Calculator:
    def __init__(self, num1=0, num2=0):
        self.num1 = num1
        self.num2 = num2
        self.result = 0
    def add(self):
        self.result = self.num1 + self.num2
    def subtract(self):
        self.result = self.num1 - self.num2
    def multiply(self):
        self.result = self.num1 * self.num2
    def divide(self):
        if self.num2 == 0:
            print("Cannot divide by zero.")
        else:
            self.result = self.num1 / self.num2
    def display(self):
        print(f"Result: {self.result}")

# Instantiate the class and test its methods
calc = Calculator(10, 5)
calc.add()
calc.display()
calc.subtract()
calc.display()
calc.multiply()
calc.display()
calc.divide()
calc.display()
