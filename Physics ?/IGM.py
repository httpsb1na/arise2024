def IGM(p):
  '''
  Input: end-effector position
  Output: joint angle
  '''

x = p[0]
y = p[1]

q = np.arctan2(y_p,x_p)
return q

# Test the DGM
p = np.array([0.3,0.5])
q = IGM(p)
print("The joint position corresponding to p = "+str(p)+" is : q = "+str(q))
