"""
Graphic interface that relays status updates from the rover.
"""

import math, time
from typing import Optional, Tuple
import json
from dataclasses import dataclass
import tkinter as tk
import sensor.shared
from sensor.shared import Coord
import sensor.metal as metal
import shared
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from PIL import Image, ImageTk
import skimage

FRAME_BORDER_WIDTH = 1
WINDOW_COLUMNS, WINDOW_ROWS = 2, 4

# in pixels
SONAR_BLOCK_RADIUS = 150
SONAR_X_OFFSET, SONAR_Y_OFFSET = 100, -20
# in inches
PLOT_SIZE = (2.5, 2.5)

METAL_PLOT_SAMPLES = 50 # TODO?
# 
METAL_PLOT_WINDOW = 10

# display image height and width in pixels (ratio 3:4)
IR_DISPLAY_WIDTH, IR_DISPLAY_HEIGHT = 800, 600

# TODO: make members optional
@dataclass
class InterfaceUpdatePacket:
    # most recent sensor readings
    # metal detector reading + timestamp
    metal: Optional[list[Tuple[int, float]]]
    sonar: Optional[list[float]]
    ircam: Optional[list[float]]

    # positions rover has visited
    path: list[sensor.shared.Coord]
    # detected mines
    mines: list[sensor.shared.Coord]
    # predicted mines
    pmines: list[sensor.shared.Coord]

def parse_packet(bytes: bytes) -> Optional[InterfaceUpdatePacket]:
    if len(bytes) > 0:
        # read header
        # sensor data minus ir-camera
        sensor_size = int.from_bytes(bytes[:int(shared.ROVER_MSG_HEADER_SIZE/2)], "little")
        # ir-camera data
        ircam_size = int.from_bytes(bytes[int(shared.ROVER_MSG_HEADER_SIZE/2):shared.ROVER_MSG_HEADER_SIZE], "little")

        # read body
        # TODO: don't we want the other data?
        sensory = json.loads(bytes[shared.ROVER_MSG_HEADER_SIZE:shared.ROVER_MSG_HEADER_SIZE+sensor_size].decode("utf-8"))
        if ircam_size > 0:
            ircam = np.frombuffer(
                bytes[shared.ROVER_MSG_HEADER_SIZE+sensor_size:shared.ROVER_MSG_HEADER_SIZE+sensor_size+ircam_size],
                dtype="float16" # TODO: variable
            )
            np.reshape(ircam, newshape=(sensor.shared.IR_IMG_SIZE))
        else:
            ircam = None

        return InterfaceUpdatePacket(
            metal=sensory.get("metal", None),
            sonar=sensory.get("sonar", None),
            ircam=ircam,
            path=sensory.get("path", []),
            mines=sensory.get("mines", []),
            pmines=sensory.get("pmines", [])
        )
    else:
        return None

# TODO: mark private members
class Interface:
    def __init__(self):
        self.__manual_control_requested = True
        self.__request_control_switch = False
    
        # value/timestamp samples are accumulated over time
        self.metal_samples = [[] for _ in range(metal.METAL_DETECTOR_COUNT)]
        self.metal_samples_times = [[] for _ in range(metal.METAL_DETECTOR_COUNT)]

        # create window
        self.window = tk.Tk("mine sniffer interface")
        # maximize screen
        self.window.state("zoomed")

        self.window.columnconfigure(list(range(WINDOW_COLUMNS)), weight=1, uniform="column")
        self.window.rowconfigure(list(range(WINDOW_ROWS)), weight=1, uniform="row")

        self.window.bind("<Key>", self.__process_key_press)
        
        self.metal_frame = tk.Frame(bg="salmon", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        # it's a canvas in order to draw laser lines
        self.sonar_frame = tk.Canvas(bg="white", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        self.ircam_frame = tk.Frame(bg="light blue", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)
        self.map_frame = tk.Frame(bg="black", relief=tk.GROOVE, borderwidth=FRAME_BORDER_WIDTH)

        self.metal_frame.grid(sticky="nsew")
        self.sonar_frame.grid(sticky="nsew")
        self.ircam_frame.grid(sticky="nsew")
        self.map_frame.grid(sticky="nsew")

        self.__set_default_frame_positions()

        # expand frame on click
        self.metal_frame.bind("<Button-1>", self.__expand_frame)
        self.sonar_frame.bind("<Button-1>", self.__expand_frame)
        self.ircam_frame.bind("<Button-1>", self.__expand_frame)
        self.map_frame.bind("<Button-1>", self.__expand_frame)

        self.__init_metal_frame()
        self.__init_sonar_frame()
        self.__init_ircam_frame()
        self.__init_map_frame()

        # for metal frame to be rendered correctly, it should happen at the end of initialization
        self.__resize_metal_frame(None)

    """ Returns `True` if a control switch has been requested. """
    def update(self, data: Optional[InterfaceUpdatePacket]) -> bool:
        if data is not None:
            self.__update_metal_frame(data)
            self.__update_sonar_frame(data)
            self.__update_ircam_frame(data)
            self.__update_map_frame(data)
        
        self.window.update()

        control_switch_requested = self.__request_control_switch
        self.__request_control_switch = False
        return control_switch_requested

    def __init_metal_frame(self):
        PLOT_COLORS = ["red", "blue", "green"]

        self.metal_frame.bind("<Configure>", self.__resize_metal_frame)

        self.metal_frame.columnconfigure(list(range(metal.METAL_DETECTOR_COUNT)), weight=1)

        metal_title = tk.Label(master=self.metal_frame, text="metal detector")
        metal_title.pack()

        self.metal_plots = []
        for n in range(metal.METAL_DETECTOR_COUNT):
            # build plot
            fig, ax = plt.subplots()
            ax.set_xlim((-METAL_PLOT_WINDOW, 0))
            ax.set_yticklabels([])

            fig.set_facecolor("salmon")
            ax.set_facecolor((0.8, 0.8, 1.0, 0.3))
            ax.grid(color=(0.8, 0.8, 1.0))

            fig.set_size_inches(*PLOT_SIZE)
            # animated is set to true because blitting is exploited to improve render times
            (line,) = ax.plot([], [], animated=True)
            line.set_color(PLOT_COLORS[n])

            # render plot
            canvas = FigureCanvasTkAgg(fig, master=self.metal_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(side="left", fill="both", expand=True)

            # the background cache will be set at the next configure event
            self.metal_plots.append((fig, ax, line, None))
        
    def __init_sonar_frame(self):
        # update lasers when frame is resized
        self.sonar_frame.bind("<Configure>", self.__resize_sonar_frame)

        sonar_title = tk.Label(master=self.sonar_frame, text="sonars")
        sonar_title.pack(pady=FRAME_BORDER_WIDTH + 2)

        # initialize sonars
        self.sonar_dists = [0.0] * sensor.shared.SONAR_COUNT
        self.sonar_labels = []
        self.sonar_lasers = []

        sonar_idx = 0
        for i in range(sensor.shared.SONAR_BLOCK_COUNT):
            angle = sensor.shared.SONAR_BLOCK_ANGLE / float(sensor.shared.SONAR_BLOCK_COUNT) * i
            x = math.cos(angle) * SONAR_BLOCK_RADIUS
            y = -math.sin(angle) * SONAR_BLOCK_RADIUS + SONAR_Y_OFFSET

            for x in [x + SONAR_X_OFFSET, -(x + SONAR_X_OFFSET)]:
                dist = self.sonar_dists[sonar_idx]

                # create distance label
                label = tk.StringVar(value=f"{dist:.0f}cm")
                tk.Label(master=self.sonar_frame, textvariable=label)\
                    .place(relx=0.5, rely=1.0, x=x, y=y, anchor=tk.CENTER)
                self.sonar_labels.append(label)
                
                # create laser. create_line does not support relative coordinates,
                #  so they need to be set at every configure event,
                #  which is first triggered when the window is displayed
                self.sonar_lasers.append(self.sonar_frame.create_line(0, 0, 0, 0, fill="red", width=2))

                sonar_idx += 1

    def __init_ircam_frame(self):
        ircam_title = tk.Label(master=self.ircam_frame, text="IR-camera")
        ircam_title.pack()

        self.img = ImageTk.PhotoImage(image=Image.fromarray(
            np.uint8(np.zeros((IR_DISPLAY_HEIGHT, IR_DISPLAY_WIDTH, 3))),
            mode="RGB"
        ))

        canvas = tk.Canvas(master=self.ircam_frame, width=IR_DISPLAY_WIDTH, height=IR_DISPLAY_HEIGHT)
        # TODO: use display size
        canvas.create_image(0, 0, anchor="nw", image=self.img)
        canvas.pack(pady=2)

    def __init_map_frame(self):
        map_title = tk.Label(master=self.map_frame, text="map")
        map_title.pack(pady=FRAME_BORDER_WIDTH + 2)

        # switch control button
        if self.__manual_control_requested:
            self.__manual_control_btn_txt = tk.StringVar(value="Change to Automatic Control")
        else:
            self.__manual_control_btn_txt = tk.StringVar(value="Change to Manual Control")

        manual_control_btn = tk.Button(
            master=self.map_frame, textvariable=self.__manual_control_btn_txt, command=self.__change_control
        )
        manual_control_btn.pack(pady=2)

        # coordinate map
        self._map_canvas = tk.Canvas(master=self.map_frame)
        self._map_canvas.pack(fill="both", expand=True, pady=2)

    def __update_metal_frame(self, data: InterfaceUpdatePacket):
        # TODO: fix y-axis tick labels
        now = time.time()
        
        # repeat for each detector
        miny, maxy = 5000, -5000 # TODO
        for n in range(metal.METAL_DETECTOR_COUNT):
            samples, times = self.metal_samples[n], self.metal_samples_times[n]

            # add new data point
            if data.metal is not None:
                samples.append(data.metal[n][0])
                times.append(data.metal[n][1])

            # remove old data points
            for i, t in enumerate(times):
                if now - t > METAL_PLOT_WINDOW:
                    samples.pop(i)
                    times.pop(i)

            if len(samples) > 0:
                miny = min([miny, min(samples)])
                maxy = max([maxy, max(samples)])

        for n in range(metal.METAL_DETECTOR_COUNT):
            fig, ax, line, bg_cache = self.metal_plots[n]
            samples = self.metal_samples[n]

            # load new data points
            line.set_data([t - now for t in times], samples)
            # rescale axes
            ax.set_ylim(miny, maxy)
            ax.autoscale_view(scalex=False)
            
            # TODO: add blit comments
            fig.canvas.restore_region(bg_cache)

            ax.draw_artist(line)
            fig.canvas.blit(fig.bbox)
            fig.canvas.flush_events()

    def __resize_metal_frame(self, _: tk.Event):
        for n in range(metal.METAL_DETECTOR_COUNT):
            (fig, ax, line, _) = self.metal_plots[n]
            # draw background
            fig.canvas.draw()
            fig.canvas.flush_events()
            # cache background
            bg_cache = fig.canvas.copy_from_bbox(fig.bbox)
            self.metal_plots[n] = (fig, ax, line, bg_cache)

    def __update_sonar_frame(self, data: InterfaceUpdatePacket):
        MAPPING = [9, 4, 8, 3, 7, 2, 6, 1, 5, 0]

        if data.sonar is not None:
            for i in range(len(data.sonar)):
                dist = data.sonar[MAPPING[i]]
                self.sonar_dists[i] = dist
                self.sonar_labels[i].set(f"{dist:.0f}cm {i}")

            self.__resize_sonar_frame()

    def __resize_sonar_frame(self, _: Optional[tk.Event] = None):
        LENGTH_MULTIPLIER = 7.5
        MAX_DISPLAY_DIST = 10000

        framew, frameh = self.sonar_frame.winfo_width(), self.sonar_frame.winfo_height()

        sonar_idx = 0
        for i in range(sensor.shared.SONAR_BLOCK_COUNT):
            angle = sensor.shared.SONAR_BLOCK_ANGLE / float(sensor.shared.SONAR_BLOCK_COUNT) * i
            c, s = math.cos(angle), math.sin(angle)
            x = c * SONAR_BLOCK_RADIUS
            y = -s * SONAR_BLOCK_RADIUS + SONAR_Y_OFFSET

            # dir = direction of the laser, guaranteed to be a unit vector
            for dir, x in [((c, -s), x + SONAR_X_OFFSET), ((-c, -s), -(x + SONAR_X_OFFSET))]: 
                if self.sonar_dists[sonar_idx] == float("inf"):
                    dist = MAX_DISPLAY_DIST
                else:
                    dist = self.sonar_dists[sonar_idx]

                length = dist * LENGTH_MULTIPLIER
                # origin and end coordinates
                x0, y0 = x + framew/2, frameh+y
                x1, y1 = x0 + dir[0]*length, y0 + dir[1]*length
                
                # update laser position
                self.sonar_frame.coords(self.sonar_lasers[sonar_idx], x0, y0, x1, y1)
                sonar_idx += 1

    def __update_ircam_frame(self, data: InterfaceUpdatePacket):
        MIN_TEMP = 20.0
        MAX_TEMP = 45.0

        COLORS = [np.array([0,0,1]), np.array([0,1,1]), np.array([0,1,0]), np.array([1,1,0]), np.array([1,0,0])]

        if data.ircam is not None:          
            small_img = data.ircam.astype("float32")

            # map onto temperature range
            small_img = np.clip((small_img - MIN_TEMP) / (MAX_TEMP - MIN_TEMP), 0.0, 0.999)
            # we map each value to a color in COLORS
            col_idx = small_img * len(COLORS)
            # however, this is a fraction, so we mix the colors on either side
            col_idx0 = np.floor(col_idx)
            col_idx1 = col_idx0 + 1
            frac = col_idx - col_idx0

            # mix # TODO: min is hacky
            col0 = np.array([COLORS[min([int(i), len(COLORS)-1])] for i in col_idx0])
            col1 = np.array([COLORS[min([int(i), len(COLORS)-1])] for i in col_idx1])
            col_img = np.array([(col1 - col0) * frac + col0 for col0, col1, frac in zip(col0, col1, frac)])[::-1]\
                .reshape((sensor.shared.IR_IMG_HEIGHT, sensor.shared.IR_IMG_WIDTH, 3))

            # upscale image
            interpolated_img = skimage.transform.resize(col_img, (IR_DISPLAY_HEIGHT, IR_DISPLAY_WIDTH))

            self.img.paste(Image.fromarray(np.uint8(interpolated_img * 255), mode="RGB"))

    def __update_map_frame(self, data: InterfaceUpdatePacket):
        PATH_COL = "green"
        MINE_COL = "red"
        PMINE_COL = "purple"
        CIRCLE_RADIUS = 10

        x_min = min([
            *[c.lat for c in data.path],
            *[c.lat for c in data.mines],
            *[c.lat for c in data.pmines]
        ], default=0.0)
        x_max = max([
            *[c.lat for c in data.path],
            *[c.lat for c in data.mines],
            *[c.lat for c in data.pmines]
        ], default=0.0)
        y_min = min([
            *[c.long for c in data.path],
            *[c.long for c in data.mines],
            *[c.long for c in data.pmines]
        ], default=0.0)
        y_max = max([
            *[c.long for c in data.path],
            *[c.long for c in data.mines],
            *[c.long for c in data.pmines]
        ], default=0.0)
        
        width, height = self._map_canvas.winfo_width(), self._map_canvas.winfo_height()
        get_pos = lambda c: ((c.lat - x_min) / (x_max - x_min) * width, (c.long - y_min) / (y_max - y_min) * height)
        
        # clear screen
        self._map_canvas.delete("all")

        # start drawing again
        draw_circle = lambda pos, col: self._map_canvas.create_oval(
            pos[0] - CIRCLE_RADIUS, pos[1] - CIRCLE_RADIUS, pos[0] + CIRCLE_RADIUS, pos[1] + CIRCLE_RADIUS,
            fill=col
        )

        for c in data.path:
            pos = get_pos(c)
            draw_circle(pos, PATH_COL)
        for c in data.mines:
            pos = get_pos(c)
            draw_circle(pos, MINE_COL)
        for c in data.pmines:
            pos = get_pos(c)
            draw_circle(pos, PMINE_COL)

    def __change_control(self):
        self.__manual_control_requested = not self.__manual_control_requested
        self.__request_control_switch = True
        
        if self.__manual_control_requested:
            self.__manual_control_btn_txt.set("Change to Automatic Control")
        else:
            self.__manual_control_btn_txt.set("Change to Manual Control")

    # TODO: rename
    def __set_default_frame_positions(self):
        self.metal_frame.grid(column=0, row=0, columnspan=1, rowspan=1)
        self.sonar_frame.grid(column=0, row=1, columnspan=1, rowspan=1)
        self.ircam_frame.grid(column=0, row=2, columnspan=1, rowspan=2)
        self.map_frame.grid(column=1, row=0, columnspan=1, rowspan=4)

    def __expand_frame(self, event: tk.Event):
        # hide all frames
        self.metal_frame.grid_remove()
        self.sonar_frame.grid_remove()
        self.ircam_frame.grid_remove()
        self.map_frame.grid_remove()

        # expand the selected frame
        event.widget.grid(column=0, row=0, columnspan=WINDOW_COLUMNS, rowspan=WINDOW_ROWS)

    def __process_key_press(self, event: tk.Event):
        if event.keysym == "Escape":
            self.__set_default_frame_positions()

    def is_window_alive(self) -> bool:
        try:
            # returns an exception if the window has been closed
            return self.window.winfo_exists()
        except: # TODO: set exception type?
            return False
        
    def manual_control_requested(self) -> bool:
        return self.__manual_control_requested

# TODO: remove
if __name__ == "__main__":
    MINES = [
        Coord(52.355786666532836, 4.956359316985091),
        Coord(52.35548468177342, 4.957062533628297),
        Coord(52.35481531591457, 4.957709450144924),
        Coord(52.354770216014025, 4.958237287577644),
        Coord(52.354274624148154, 4.9590106351523175),
        Coord(52.35377523782767, 4.95989568527008),
        Coord(52.356451702866345, 4.956711409165704),
        Coord(52.35617304766104, 4.957392919807782),
        Coord(52.355803410564384, 4.957973366554959),
        Coord(52.355435088989985, 4.958618905582301),
        Coord(52.35491548250594, 4.959555419099146),
        Coord(52.35461985484739, 4.9601533748397895),
        Coord(52.35719578086139, 4.957305342158643),
        Coord(52.3566635731529, 4.957761480069876),
        Coord(52.356206835314445, 4.958439995722376),
        Coord(52.355879864778196, 4.959257170655912),
        Coord(52.35553046049057, 4.959830088970801),
        Coord(52.35516839928895, 4.960561343834423),
        Coord(52.35774867156988, 4.957533226171882),
        Coord(52.35747105701092, 4.958253888443774),
        Coord(52.35711885098245, 4.958994478250333),
        Coord(52.356495008301266, 4.959591577465827),
        Coord(52.356217369392304, 4.960461666775987),
        Coord(52.35589248157915, 4.9609805385030405),
        Coord(52.35860580291784, 4.95804885050571),
        Coord(52.35831222785892, 4.958683872265238),
        Coord(52.35779337614096, 4.9593849484344075),
        Coord(52.35745288523362, 4.959982399564932),
        Coord(52.356945219068066, 4.960762341938058),
        Coord(52.35654451004963, 4.961614051895064),
        Coord(52.359293596912586, 4.958194302229437),
        Coord(52.35890860276109, 4.9589396612523835),
        Coord(52.358451219022584, 4.959786720068025),
        Coord(52.358192265733166, 4.960320679498525),
        Coord(52.357569866779784, 4.961028663642502),
        Coord(52.35728017497697, 4.961779489112412)
    ]

    PMINES = [
        Coord(52.35347575186776, 4.951979798887772),
        Coord(52.35312895267242, 4.952436868229808), 
        Coord(52.353031312443626, 4.95321893980768), 
        Coord(52.35252673887984, 4.953970740372657), 
        Coord(52.352458370918505, 4.9547281057881545), 
        Coord(52.35219795216362, 4.955501484783332), 
        Coord(52.351958502529406, 4.956157081177867), 
        Coord(52.354288124127635, 4.95205787704335), 
        Coord(52.353961698491396, 4.952808992311943), 
        Coord(52.353539479597046, 4.953591962729513), 
        Coord(52.35354728578459, 4.954305525847587), 
        Coord(52.35326468268673, 4.955163399510461),
        Coord(52.35295175268932, 4.955768009002355),
        Coord(52.35268033831636, 4.9565310107312115),
        Coord(52.35496920296571, 4.952482669822929), 
        Coord(52.354692049395474, 4.953130357773565), 
        Coord(52.35456679872145, 4.953929082231458), 
        Coord(52.35413179814544, 4.954449304900906), 
        Coord(52.35393890101658, 4.955330648610965), 
        Coord(52.353799556253584, 4.956008464448044), 
        Coord(52.35345130259131, 4.956901322690194), 
        Coord(52.35575625583956, 4.952578872543698), 
        Coord(52.355351234465736, 4.953579068584111), 
        Coord(52.355073559672064, 4.954043538234803), 
        Coord(52.35484573119636, 4.95474322910694), 
        Coord(52.35453424464083, 4.955583437581818),
        Coord(52.354411963356114, 4.956333375868191),
        Coord(52.353976524862645, 4.957034420540683),
        Coord(52.35629907730934, 4.952845533507228),
        Coord(52.355989206625864, 4.9535442003363555),
        Coord(52.35576306932015, 4.954368465141929),
        Coord(52.35557430508064, 4.95512100134534),
        Coord(52.35524697421447, 4.95596185165165),
        Coord(52.35521367387504, 4.956598710882358),
        Coord(52.354837648107086, 4.957400974625689),
        Coord(52.356996329864906, 4.952989713570777),
        Coord(52.35691943469158, 4.953909050028631), 
        Coord(52.356551683828435, 4.954533020546861),
        Coord(52.35648506098545, 4.955380767103172),
        Coord(52.35602600824348, 4.956082958120226),
        Coord(52.35591956082229, 4.956727839909744),
        Coord(52.35549257684131, 4.9574172805349646),
        Coord(52.35780559664365, 4.953284240902315),
        Coord(52.357526865280406, 4.954130251501447),
        Coord(52.35730467214571, 4.954982414634617),
        Coord(52.35691916501353, 4.955644147607901),
        Coord(52.35688846665012, 4.956232744570456),
        Coord(52.35637269190923, 4.957199920212233),
        Coord(52.356248456915544, 4.957826235153093)
    ]

    interface = Interface()

    # TODO: weg
    last_frame_time = last_data_update_time = time.time()

    # TODO: manage some fps?
    while interface.is_window_alive():
        # TODO: accept sensor data over TCP
        now = time.time()

        delta = now - last_frame_time
        if delta != 0:
            print(f"fps: {1.0 / delta}")
        last_frame_time = now

        if now - last_data_update_time > 0.1:
            sensor_data = InterfaceUpdatePacket(
                metal=[(int(math.cos(now * 5) * 100), now)] * metal.METAL_DETECTOR_COUNT,
                sonar=[50.0 + math.sin(now) * 20] * sensor.shared.SONAR_COUNT,
                ircam=None,
                path=[],
                mines=MINES,
                pmines=PMINES
            )
            last_data_update_time = now
        else:
            sensor_data = None

        interface.update(sensor_data)
