# Implement a function to find the second largest number in a list. Test it.

def a(list_a):
    if len(list_a) < 2:
        return "List must contain at least two elements"
    list_a.sort()
    return list_a[-2]
  
  

