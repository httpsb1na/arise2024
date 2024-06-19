# Write a function to find the factors of a positive number. Test the function on several examples.

def b(x):
  factors = [] # ARRAYS, to hold the all the postive numbers !!!
  for i in range(1, n+1): # Looping through all postive numbers 
    if n % i == 0: # If there is no reminder then...
      factors.append(i) # Add to the array
    return factors; # Return the array !!!
