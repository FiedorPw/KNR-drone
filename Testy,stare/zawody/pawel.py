import time

i=0

while True:
    if (i%3==0 and i%5==0):
        print("FizzBuzz")
    elif (i%5==0):
        print("Buzz")
    elif (i%3==0):
        print("Fizz")
    else:
        print(i)
    i+=1
    time.sleep(1)

    


