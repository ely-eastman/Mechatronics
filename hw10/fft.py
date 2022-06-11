from ulab import numpy as np
import time
arr = np.array(range(1024))*2*np.pi/1024
print(arr)
sin_arr = np.sin(300*arr) + np.sin(600*arr) + np.sin(1500*arr)
print(sin_arr)
a, b = np.fft.fft(sin_arr)
print(a)
print(b)
spectrum = a**2 + b**2
print(spectrum)
for i in spectrum:
    print("("+str(i)+",)")
    time.sleep(0.1)
