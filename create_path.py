file = open("path.txt", 'w');

x = 0
y = 0
for i in range(100):
    x +=1
    y +=1
    file.write("{},{}\n".format(x,y))
    # print("{},{}\n".format(x,y))
for i in range(100):
    x -=1
    y +=1
    file.write("{},{}\n".format(x,y))
    # print("{},{}\n".format(x,y))


file.close()
