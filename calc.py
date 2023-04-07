def main():
    print("Let's implement addition, multiple. Type two numbers for x and y.")

    x = int(input("x > "))
    y = int(input("y > "))
    
    print("%d + %d = %d " % (x, y, add(x, y)))   
    print("%d * %d = %d " % (x, y, multiple(x, y)))   
      

<<<<<<< HEAD
#daadfsfas
=======
>>>>>>> 8b23ce9b9a15a2e111335bd10668e6ab9ae65e5d
# TODO: add() 함수 정의
def add(x,y):
    return x+y
    
def multiple(x,y):
	return x*y

if __name__ == "__main__":
    main()
    
