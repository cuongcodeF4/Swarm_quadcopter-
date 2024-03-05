import subprocess

# Đường dẫn đến file Python mà bạn muốn chạy
python_file = "/home/datltp/ardupilot/Tools/autotest/sim_vehicle.py"

# Các tham số đầu vào bạn muốn truyền
params = ["-vArduCopter", "--console", "--map"]

# Sử dụng subprocess để chạy file Python với các tham số đầu vào
subprocess.run(["python", python_file] + params)

