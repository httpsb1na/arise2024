# Define the ranges and functions for all three plots
x_sin = np.linspace(0, 2 * np.pi, 1000)
y_sin = np.sin(x_sin)

x_exp = np.linspace(0, 5, 1000)
y_exp = np.exp(x_exp)

x_poly = np.linspace(-2, 4, 1000)
y_poly = np.power(x_poly, 3) - 6 * np.power(x_poly, 2) + 11 * x_poly - 6

# Plot all three functions on the same graph
plt.figure()
plt.plot(x_sin, y_sin, label='sin(x)')
plt.plot(x_exp, y_exp, label='exp(x)')
plt.plot(x_poly, y_poly, label='x^3 - 6x^2 + 11x - 6')

# Add labels, title, legend, and grid
plt.xlabel('x')
plt.ylabel('y')
plt.title('Comparison of sin(x), exp(x), and x^3 - 6x^2 + 11x - 6')
plt.legend()
plt.grid(True)
plt.show()
