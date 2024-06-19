# Problem 01 
# Write a function that returns the division of 2 numbers. If the denominator is 0, the function should print an error message and return nothing. Test the function on several examples.

def x(a,b): 
  if(b != 0):
    return a/b
  else: 
    print("Denominator cannot be 0, adjust it.")
