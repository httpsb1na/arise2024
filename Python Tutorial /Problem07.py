# Question : Plot the exponential function for -2 <= x <= 5 with 1000 points

# Define the range and the polynomial function
x = t.linspace(-2, 5, 1000)
y = t.power(x, 3) - 6 * t.power(x, 2) + 11 * x - 6

# Plot the polynomial function
m.figure()
m.plot(x, y)
m.xlabel('x')
m.ylabel('y')
m.title('Plot of the polynomial x^3 - 6x^2 + 11x - 6 from -2 to 5')
m.grid(True)
m.show()
