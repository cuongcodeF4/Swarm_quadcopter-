import threading
import time

# Biến cờ để kiểm tra khi nào thoát khỏi luồng
exit_flag = False

def thread_function_1():
    for i in range(5):
        print("Thread 1 is running")
        time.sleep(1)

def thread_function_2():
    global exit_flag
    for i in range(5):
        print("Thread 2 is running")
        if i == 3:
            exit_flag = True  # Đặt cờ để thoát khỏi luồng
            break
        time.sleep(1)

if __name__ == "__main__":
    # Tạo các đối tượng thread
    thread1 = threading.Thread(target=thread_function_1)
    thread2 = threading.Thread(target=thread_function_2)

    # Bắt đầu thực thi các luồng
    thread1.start()
    thread2.start()
    print("A")
    # Đợi cho tất cả các luồng kết thúc
    thread1.join()
    thread2.join()

    print("Main thread ends")
