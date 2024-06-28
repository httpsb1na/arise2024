# Plot y = sin(x) with 1000 points. 

import matplotlib.pyplot as s
import numpy as o

# Define the range and the sine function
x = o.linspace(0, 2 * o.pi, 1000)
y = o.sin(x)

# Plot the sine function
s.figure()
s.plot(x, y)
s.xlabel('x')
s.ylabel('sin(x)')
s.title('Plot of sin(x) from 0 to 2π')
s.grid(True)
s.show()
