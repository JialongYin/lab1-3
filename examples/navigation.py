import picar_4wd as fc
import time

speed = 30

def main():
    start = time.time()
    i = 0
    while True:
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue

        tmp = scan_list[3:7]
        print(tmp)
        if tmp != [2,2,2,2]:
            fc.backward(speed)
            time.sleep(0.1)
            fc.stop()

            if i % 2 == 0:
                fc.turn_right(speed)
            else:
                fc.turn_left(speed)
            time.sleep(0.4)
            i += 1
        else:
            fc.forward(speed)
            if (time.time() - start) > 10:
                break

if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()
