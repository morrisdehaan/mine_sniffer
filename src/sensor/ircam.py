# import adafruit_mlx90640

# import board
# import busio
# import numpy as np

# IRCAM_IMG_SIZE = 768

# class IRCam:
#     def __init__(self):
#         i2c = busio.I2C(board.SCL, board.SDA)

#         self.mlx = adafruit_mlx90640.MLX90640(i2c)
#         self.mlx.refresh_rate(adafruit_mlx90640.RefreshRate.REFRESH_2_HZ)

#         self.raw = np.zeros(IRCAM_IMG_SIZE)

#     def update(self):
#         try:
#             self.mlx.getFrame(self.raw)
#         except ValueError:
#             return