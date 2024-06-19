# Quiz 01

# function that prints the input
def myPrintFunc(x):
    print('x =', x)

# calling the functions defined above.
myPrintFunc(42)

    
# function that adds two input numbers
def myFunc(x, y):
    return x + y

sum_value = myFunc(1, 2)
print('The sum of 1 + 2 is', sum_value)

def myFunc2(x, y, show=True):
    w = myFunc(x, y)
    if(show):
        myPrintFunc(w)
    return w

# The following code adds the variables 'x' and 'y' to create 'w'. The result 'w' is solved regardless, but it will only be returned to the user if 'show' is set to True. Otherwise, 'w' is not provided to the user. The 'show' variable behaves by displaying the result to the user.



