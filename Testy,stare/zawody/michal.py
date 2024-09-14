import time
counter = 0
while True:
    time.sleep(1)
    counter+=1
    if counter%3==0 or counter%5==0:
        if counter%3==0:
            print("fizz")
        if counter%5==0:
            print("bizz")
        if counter%3==0 and counter%5==0:
            print("fizzbizz")
    else:
        print(counter)

    