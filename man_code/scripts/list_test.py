

l = (1,2,3,4)
l= list(l)
print(l)
print(l[0])
fake= l[0]
l = l[1:]
l.append(fake)

print(l)
