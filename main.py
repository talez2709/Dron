from djitellopy import tello
import keyboard
import os
import cv2
import numpy as np
import time
import csv
import copy
import face_detection
import mediapipe as mp
import math
from typing import Tuple, Union
from simple_pid import PID
import datetime as dt

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
from mtcnn.mtcnn import MTCNN
from elliottzheng_face_detection.face_detection import RetinaFace
from dataclasses import dataclass, field
import xxhash
from yoloface import face_analysis
from threading import *
import advanced_pid
from datetime import datetime


class TelloController:
    class TelloKillSwitch(Thread):

        tc_handler = None

        def __init__(self, tc_handler):
            Thread.__init__(self)
            self.tc_handler = tc_handler

        def run(self):
            keyboard.wait('space')
            self.tc_handler.force_emergency_stop()

    class TelloTimer(Thread):
        interval = 1.0
        running = None
        func = None

        def __init__(self, interval, event, func):
            Thread.__init__(self)
            self.running = event
            self.interval = interval
            self.func = func

        def run(self):
            while not self.running.wait(self.interval):
                self.func()

    class TelloNoTimer(Thread):
        running = None
        func = None

        def __init__(self, event, func):
            Thread.__init__(self)
            self.running = event
            self.func = func

        def run(self):
            self.func()

    tello_drone = None
    stop_controller = None

    def force_emergency_stop(self):
        self.tello_drone.emergency()
        self.stop_controller.set()

    def printBattery(self):
        print('Battery:', str(float(self.tello_drone.get_battery())), "% Temp:", self.tello_drone.get_temperature(),
              "°C")

    def BatteryLvl(self):
        bateria = self.tello_drone.get_battery()
        if bateria < 20:
            print("NISKI POZIOM NAŁADOWANIA BATERII, < 20%")
            if self.dron_w_powietrzu:
                self.tello_drone.land()
                self.dron_w_powietrzu = False
        elif bateria < 35:
            print("NISKI POZIOM NAŁADOWANIA BATERII, < 35%")
            if self.dron_w_powietrzu:
                self.tello_drone.land()
                self.dron_w_powietrzu = False

    nazwa_pliku = 'Log/' + datetime.now().strftime("%Y-%m-%d %H-%M-%S") + " pokladowy.csv"

    @dataclass
    class Telemetry:
        datestamp: str = field(default_factory=str)
        accX: float = field(default_factory=float)
        accY: float = field(default_factory=float)
        accZ: float = field(default_factory=float)
        Vx: int = field(default_factory=int)
        Vy: int = field(default_factory=int)
        Vz: int = field(default_factory=int)
        roll: float = field(default_factory=float)
        pitch: float = field(default_factory=float)
        yaw: float = field(default_factory=float)
        time: float = field(default_factory=int)
        battery: int = field(default_factory=int)
        barometer: float = field(default_factory=float)
        height: int = field(default_factory=int)
        lTemp: int = field(default_factory=int)
        hTemp: int = field(default_factory=int)
        Temp: float = field(default_factory=float)
        mission_pad_id: int = field(default_factory=int)
        mission_pad_distance_x: int = field(default_factory=int)
        mission_pad_distance_y: int = field(default_factory=int)
        mission_pad_distance_z: int = field(default_factory=int)
        tof: int = field(default_factory=int)
        state: dict = field(default_factory=dict)

    Czujniki = Telemetry()

    def ImportTelemetry(self):
        a = self.tello_drone.get_own_udp_object()
        self.Czujniki.state = self.tello_drone.get_current_state()
        self.Czujniki.datestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        self.Czujniki.accX = self.Czujniki.state['agx']
        self.Czujniki.accY = self.Czujniki.state['agy']
        self.Czujniki.accZ = self.Czujniki.state['agz']
        self.Czujniki.Vx = self.Czujniki.state['vgx']
        self.Czujniki.Vy = self.Czujniki.state['vgy']
        self.Czujniki.Vz = self.Czujniki.state['vgz']
        self.Czujniki.roll = self.Czujniki.state['roll']
        self.Czujniki.pitch = self.Czujniki.state['pitch']
        self.Czujniki.yaw = self.Czujniki.state['yaw']
        self.Czujniki.time = self.Czujniki.state['time']
        self.Czujniki.battery = self.Czujniki.state['bat']
        self.Czujniki.barometer = self.Czujniki.state['baro'] * 100
        self.Czujniki.height = self.Czujniki.state['h']
        self.Czujniki.lTemp = self.Czujniki.state['templ']
        self.Czujniki.hTemp = self.Czujniki.state['temph']
        self.Czujniki.Temp = (self.Czujniki.lTemp + self.Czujniki.hTemp) / 2
        if self.dron_sdk == '20' or self.dron_sdk == '30':
            self.Czujniki.mission_pad_id = self.Czujniki.state['mid']
            self.Czujniki.mission_pad_distance_x = self.Czujniki.state['x']
            self.Czujniki.mission_pad_distance_y = self.Czujniki.state['y']
            self.Czujniki.mission_pad_distance_z = self.Czujniki.state['z']
        if self.dron_sdk == '30':
            self.Czujniki.tof = self.Czujniki.state['tof']

    def SaveToFileHeader(self):
        if not os.path.exists('Log'):
            os.mkdir('Log')
        with open(TelloController.nazwa_pliku, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            dane = ['datestamp', 'accX', 'accY', 'accZ', 'Vx', 'Vy', 'Vz', 'roll', 'pitch', 'yaw', 'time',
                    'battery',
                    'barometer',
                    'height', 'lTemp', 'hTemp', 'Temp']
            if self.dron_sdk == '20' or self.dron_sdk == '30':
                dane = [dane, 'mission_pad_id', 'mission_pad_distance_x',
                        'mission_pad_distance_y', 'mission_pad_distance_z']
            if self.dron_sdk == '30':
                dane = [dane, 'tof']
            csvwriter.writerow(dane)

    def SaveToFile(self):
        if self.Czujniki.datestamp != "":
            with open(TelloController.nazwa_pliku, 'a', newline='') as csvfile:
                csvwriter = csv.writer(csvfile, dialect='unix')  # unix, excel, excel_tab, unix_dialect
                dane = [self.Czujniki.datestamp,
                        self.Czujniki.accX,
                        self.Czujniki.accY,
                        self.Czujniki.accZ,
                        self.Czujniki.Vx,
                        self.Czujniki.Vy,
                        self.Czujniki.Vz,
                        self.Czujniki.roll,
                        self.Czujniki.pitch,
                        self.Czujniki.yaw,
                        self.Czujniki.time,
                        self.Czujniki.battery,
                        self.Czujniki.barometer,
                        self.Czujniki.height,
                        self.Czujniki.lTemp,
                        self.Czujniki.hTemp,
                        self.Czujniki.Temp]
                if self.dron_sdk == '20' or self.dron_sdk == '30':
                    dane = [dane,
                            self.Czujniki.mission_pad_id,
                            self.Czujniki.mission_pad_distance_x,
                            self.Czujniki.mission_pad_distance_y,
                            self.Czujniki.mission_pad_distance_z]
                if self.dron_sdk == '30':
                    dane = [dane,
                            self.Czujniki.tof]
                csvwriter.writerow(dane)

    ArtificialHorizon_time = time.time_ns()

    def ArtificialHorizon(self):
        Obraz = copy.copy(self.Obraz)
        if len(Obraz) == 0:
            Obraz = 255 * np.ones(shape=[256, 512, 3], dtype=np.uint8)
        szerokosc = int(Obraz.shape[1] / 2)
        wysokosc = int(Obraz.shape[0] / 2)
        Obraz = cv2.resize(Obraz, (szerokosc, wysokosc))

        procent_kresek = 0.1 * 2
        procent_przerwy = 0.1
        if (procent_kresek + procent_kresek) > 1.0:
            procent_kresek = procent_kresek / (1.0 / (procent_kresek + procent_kresek))
            procent_przerwy = procent_przerwy / (1.0 / (procent_kresek + procent_kresek))

        procent_wysokosci = 0.05
        if procent_wysokosci > 0.5:
            procent_wysokosci = 0.5

        kolor_staly = (128, 128, 128)
        kolor_zmienny = (32, 64, 128)

        pitch = self.Czujniki.pitch
        roll = -self.Czujniki.roll
        yaw = 0

        max_pitch = 90
        pitch_korekta = -(pitch / max_pitch) * (wysokosc / 2.0)

        max_yaw = 90
        yaw_korekta = -(yaw / max_yaw) * (szerokosc / 2.0)

        a = (procent_kresek + procent_przerwy) * szerokosc
        b = szerokosc - a
        c = int(b / 2.0)
        d = int(c + (procent_kresek / 2.0) * szerokosc)

        e = int(d + (procent_przerwy * szerokosc))
        f = int(e + (procent_kresek / 2.0) * szerokosc)

        g = int(d + (procent_przerwy / 3) * szerokosc)

        h = int(d + (procent_przerwy / 3) * szerokosc + (procent_przerwy / 3 / 2) * szerokosc)
        i = (wysokosc / 2) + (wysokosc * procent_wysokosci)

        j = int(h + (procent_przerwy / 3 / 2) * szerokosc)

        k = int(j + (procent_przerwy / 3) * szerokosc)

        abc = cv2.getRotationMatrix2D(center=(int(szerokosc / 2), int(wysokosc / 2)),
                                      angle=roll, scale=1)

        def transformacja(punkt, macierz, x_przesuniecie=0, y_przesuniecie=0):
            x, y = punkt
            return (int(macierz[0, 0] * x + macierz[0, 1] * y + macierz[0, 2] + x_przesuniecie),
                    int(macierz[1, 0] * x + macierz[1, 1] * y + macierz[1, 2] + y_przesuniecie))

        cv2.line(Obraz, transformacja((d, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta),
                 transformacja((g, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta), kolor_zmienny, 1)
        cv2.line(Obraz, transformacja((g, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta),
                 transformacja((h, int(i)), abc, yaw_korekta, pitch_korekta), kolor_zmienny, 1)
        cv2.line(Obraz, transformacja((h, int(i)), abc, yaw_korekta, pitch_korekta),
                 transformacja((j, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta), kolor_zmienny, 1)
        cv2.line(Obraz, transformacja((j, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta),
                 transformacja((k, int(wysokosc / 2)), abc, yaw_korekta, pitch_korekta), kolor_zmienny, 1)

        cv2.line(Obraz, (c, int(wysokosc / 2)), (d, int(wysokosc / 2)), kolor_staly, 1)
        cv2.line(Obraz, (e, int(wysokosc / 2)), (f, int(wysokosc / 2)), kolor_staly, 1)

        cv2.putText(Obraz, str(int(1e+9 / (time.time_ns() - self.ArtificialHorizon_time))), (szerokosc - 25, 12),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5,
                    (255, 0, 0), 1)
        self.ArtificialHorizon_time = time.time_ns()

        cv2.imshow('ArtificialHorizon', Obraz)
        cv2.waitKey(1)

    ArtificialHorizon2_time = time.time_ns()

    def ArtificialHorizon2(self):
        szerokosc = 256
        wysokosc = 512
        Img = 255 * np.ones(shape=[wysokosc, szerokosc, 3], dtype=np.uint8)

        Img = cv2.rectangle(Img, (0, 0), (szerokosc, int(wysokosc / 2)), (255, 105, 65), -1)
        Img = cv2.rectangle(Img, (0, int(wysokosc / 2)), (szerokosc, wysokosc), (48, 64, 110), -1)
        Img = cv2.line(Img, (0, int(wysokosc / 2)), (szerokosc, int(wysokosc / 2)), (255, 255, 255), 1)

        max_pitch = 90
        ilosc_kresek = int(max_pitch / 5) + 1
        dzielenie_kreski = 4 * 2
        for i in range(-ilosc_kresek + 1, ilosc_kresek):
            if i != 0:
                if i % 2 == 0:
                    Img = cv2.line(Img, (int(szerokosc / 2 - szerokosc / dzielenie_kreski),
                                         int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * i))), (
                                       int(szerokosc / 2 + szerokosc / dzielenie_kreski),
                                       int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * i))),
                                   (255, 255, 255), 1)
                    Img = cv2.putText(Img, str(abs(i * 5)), (int(szerokosc / 2 + szerokosc / dzielenie_kreski),
                                                             int(int(wysokosc / 2) - (
                                                                     int(wysokosc / 2) / ilosc_kresek * i)) + 5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    Img = cv2.line(Img, (int(szerokosc / 2 - szerokosc / dzielenie_kreski / 2),
                                         int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * i))), (
                                       int(szerokosc / 2 + szerokosc / dzielenie_kreski / 2),
                                       int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * i))),
                                   (255, 255, 255), 1)
                    Img = cv2.putText(Img, str(abs(i * 5)), (int(szerokosc / 2 + szerokosc / dzielenie_kreski / 2),
                                                             int(int(wysokosc / 2) - (
                                                                     int(wysokosc / 2) / ilosc_kresek * i)) + 5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        pitch = self.Czujniki.pitch
        roll = -self.Czujniki.roll
        yaw = 0

        rotated_image = cv2.warpAffine(src=Img,
                                       M=cv2.getRotationMatrix2D(center=(int(szerokosc / 2), int(wysokosc / 2)),
                                                                 angle=roll, scale=1), dsize=(szerokosc, wysokosc))
        max_yaw = 90
        yaw_korekta = -(yaw / max_yaw) * (szerokosc / 2.0)
        rotated_image = cv2.warpAffine(src=rotated_image,
                                       M=np.float32([[1, 0, yaw_korekta], [0, 1, 0]]), dsize=(szerokosc, wysokosc))
        dzielenie_kreski_2 = 0.9
        dzielenie_kreski_3 = 3
        a = int((1 / dzielenie_kreski_2) * int(szerokosc / 2 - szerokosc / dzielenie_kreski))
        b = int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * (pitch / 5)))
        c = int(int(szerokosc / 2 + szerokosc / dzielenie_kreski) + int(
            (1 - 1 / dzielenie_kreski_2) * int(szerokosc / 2 - szerokosc / dzielenie_kreski)))
        d = int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * (pitch / 5)))
        rotated_image = cv2.line(rotated_image, (a, b),
                                 (c - int((c - a) * ((dzielenie_kreski_3 - 1) / dzielenie_kreski_3)), d), (255, 0, 255),
                                 1)
        rotated_image = cv2.line(rotated_image, (a + int((c - a) * ((dzielenie_kreski_3 - 1) / dzielenie_kreski_3)), b),
                                 (c, d), (255, 0, 255), 1)
        nadatek_kresek = 7
        a = int(int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * ((pitch / 5) + nadatek_kresek))))
        b = int(int(int(wysokosc / 2) - (int(wysokosc / 2) / ilosc_kresek * ((pitch / 5) - nadatek_kresek))))
        c = 0
        d = szerokosc
        if a < 0:
            a = 0
        if b < 0:
            b = 0
        cropped_image = rotated_image[a:b, c:d]

        cv2.putText(cropped_image, str(int(1e+9 / (time.time_ns() - self.ArtificialHorizon2_time))),
                    (cropped_image.shape[1] - 25, 12),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5,
                    (255, 0, 0), 1)
        self.ArtificialHorizon2_time = time.time_ns()

        cv2.imshow('ArtificialHorizon2', cropped_image)
        cv2.waitKey(1)

    Obraz = []
    Obraz_f_fps = 0
    Obraz_f_time = time.time_ns()
    Obraz_f_poprzedni_hash = 0

    def Obraz_f(self):
        self.tello_drone.streamon()
        frame_read = self.tello_drone.get_frame_read()
        keepRecording = True

        while keepRecording:
            if frame_read.frame is not None:
                if (xxhash.xxh3_64_digest(frame_read.frame) != self.Obraz_f_poprzedni_hash):
                    self.Obraz = frame_read.frame
                    self.Obraz_f_poprzedni_hash = xxhash.xxh3_64_digest(frame_read.frame)

                    self.Obraz_f_fps = int(1e+9 / (time.time_ns() - self.Obraz_f_time))
                    self.Obraz_f_time = time.time_ns()
                    time.sleep(1 / 60)

    Semaphore_Twarze = Semaphore(1)

    class Twarze_class:
        obraz = np.empty(0, dtype=np.int32)

        def __init__(self):
            self.pozycje = []
            self.ilosc = 0
            self.pozycja_srodka = np.empty(0)
            self.metoda = ''
            self.granice = np.empty(0)
            self.pozycja_granice_srodka = np.empty(0)

        def __str__(self):
            return f"{self.ilosc}({self.metoda})"

        def dodaj(self, x, y, x1, y1, metoda):
            if self.metoda != metoda:
                self.wyczysc()
            self.pozycje.append([x, y, x1, y1])
            self.ilosc += 1
            self.metoda = metoda
            self.wylicz_srodek_srodek()
            self.wylicz_granice()
            self.wylicz_granice_srodek()

        @staticmethod
        def wylicz_srodek(arr):
            x, y, x1, y1 = [i for i in arr]
            x_srodek = (x + x1) / 2
            y_srodek = (y + y1) / 2
            return [x_srodek, y_srodek]

        def wylicz_srodek_srodek(self):

            suma = [0, 0]
            for i in self.pozycje:
                suma = np.add(suma, self.wylicz_srodek(i))

            srednia = np.divide(suma, [len(self.pozycje), len(self.pozycje)])
            self.pozycja_srodka = srednia
            return srednia

        def wylicz_granice(self):
            if not self.ilosc:
                granice = [0, 0]
            else:
                x, y, x1, y1 = [i for i in self.pozycje[0]]
                granice = [[x, y], [x1, y1]]
                for arr in self.pozycje:
                    x, y, x1, y1 = [i for i in arr]
                    g, g1 = [i for i in granice]
                    x_g, y_g = [i for i in g]
                    x1_g, y1_g = [i for i in g1]
                    if x < x_g:
                        x_g = x
                    if x1 > x1_g:
                        x1_g = x1
                    if y < y_g:
                        y_g = y
                    if y1 > y1_g:
                        y1_g = y1
                    granice = [[x_g, y_g], [x1_g, y1_g]]
            self.granice = granice
            return granice

        def wylicz_granice_srodek(self):
            srodek = self.wylicz_srodek([self.granice[0][0], self.granice[0][1], self.granice[1][0],
                                         self.granice[1][1]])
            self.pozycja_granice_srodka = srodek
            return srodek

        def wyczysc(self):
            self.pozycje = []
            self.ilosc = 0
            self.pozycja_srodka = np.empty(0)
            self.metoda = ''
            self.granice = np.empty(0)
            self.pozycja_granice_srodka = np.empty(0)

    twarze = Twarze_class()

    metoda = 3
    Detekcja_poprzedni_hash = 0

    def Detekcja(self):
        detector_1 = face_detection.build_detector("DSFDDetector", confidence_threshold=.5, nms_iou_threshold=.3)
        face_cascade_2 = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        face_detection_3 = mp_face_detection.FaceDetection(
            model_selection=1, min_detection_confidence=0.25)
        detector_4 = MTCNN()
        detector5 = RetinaFace()
        detector6 = face_analysis()
        while 1:
            if len(self.Obraz) != 0:
                if (xxhash.xxh3_64_digest(self.Obraz) != self.Detekcja_poprzedni_hash):
                    self.Detekcja_poprzedni_hash = xxhash.xxh3_64_digest(self.Obraz)

                    faces = copy.copy(self.Obraz)

                    if self.metoda == 1:
                        detections = detector_1.detect(faces[:, :, ::-1])[:, :5]
                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)
                        if len(detections):
                            for face in detections:
                                x, y, x1, y1, conf = [int(i) for i in face]
                                self.twarze.dodaj(x, y, x1, y1, 'DSFD')
                        self.Semaphore_Twarze.release()
                    elif self.metoda == 2:
                        detections = face_cascade_2.detectMultiScale(cv2.cvtColor(faces, cv2.COLOR_BGR2GRAY), 1.3, 5)

                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)
                        for (x, y, w, h) in detections:
                            self.twarze.dodaj(x, y, x + w, y + h, 'haarcascade')
                        self.Semaphore_Twarze.release()

                    elif self.metoda == 3:
                        results = face_detection_3.process(cv2.cvtColor(faces, cv2.COLOR_BGR2RGB))
                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)
                        if results.detections:
                            for detection in results.detections:

                                def _normalized_to_pixel_coordinates(
                                        normalized_x: float, normalized_y: float, image_width: int,
                                        image_height: int) -> Union[None, Tuple[int, int]]:
                                    """Converts normalized value pair to pixel coordinates."""

                                    # Checks if the float value is between 0 and 1.
                                    def is_valid_normalized_value(value: float) -> bool:
                                        return (value > 0 or math.isclose(0, value)) and (value < 1 or
                                                                                          math.isclose(1, value))

                                    if not (is_valid_normalized_value(normalized_x) and
                                            is_valid_normalized_value(normalized_y)):
                                        return None
                                    x_px = min(math.floor(normalized_x * image_width), image_width - 1)
                                    y_px = min(math.floor(normalized_y * image_height), image_height - 1)
                                    return x_px, y_px

                                relative_bounding_box = detection.location_data.relative_bounding_box
                                rect_start_point = _normalized_to_pixel_coordinates(
                                    relative_bounding_box.xmin, relative_bounding_box.ymin, faces.shape[1],
                                    faces.shape[0]
                                )
                                rect_end_point = _normalized_to_pixel_coordinates(
                                    relative_bounding_box.xmin + relative_bounding_box.width,
                                    relative_bounding_box.ymin + relative_bounding_box.height, faces.shape[1],
                                    faces.shape[0]
                                )
                                if rect_start_point is not None and rect_end_point is not None:
                                    self.twarze.dodaj(rect_start_point[0], rect_start_point[1], rect_end_point[0],
                                                      rect_end_point[1], 'MediaPipe Face Detection')

                        self.Semaphore_Twarze.release()
                    elif self.metoda == 4:
                        detections = detector_4.detect_faces(cv2.cvtColor(faces, cv2.COLOR_BGR2RGB))
                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)
                        for face in detections:
                            (x, y, w, h) = face['box']
                            self.twarze.dodaj(x, y, x + w, y + h, 'MTCNN')
                        self.Semaphore_Twarze.release()
                    elif self.metoda == 5:
                        detections = detector5(cv2.cvtColor(faces, cv2.COLOR_BGR2RGB))
                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)

                        for (box, landmarks, score) in detections:
                            if score >= 0.55:
                                self.twarze.dodaj(box[0], box[1], box[2], box[3], 'RetinaFace')
                        self.Semaphore_Twarze.release()
                    elif self.metoda == 6:
                        _, box, conf = detector6.face_detection(frame_arr=faces, model='tiny')
                        self.Semaphore_Twarze.acquire()
                        self.twarze.wyczysc()
                        self.twarze.obraz = copy.copy(faces)
                        for i in range(len(conf)):
                            if conf[i] >= 0.55:
                                (x, y, w, h) = box[i]
                                self.twarze.dodaj(x, y, x + w, y + h, 'Yolo3')
                        self.Semaphore_Twarze.release()

    Detekcja_rysowanie_time = time.time_ns()
    Detekcja_rysowanie_poprzedni_hash = 0

    def Detekcja_rysowanie(self):
        if (xxhash.xxh3_64_digest(self.twarze.obraz) != self.Detekcja_rysowanie_poprzedni_hash):
            self.Detekcja_rysowanie_poprzedni_hash = xxhash.xxh3_64_digest(self.twarze.obraz)
            self.Semaphore_Twarze.acquire()
            twarze_kopia = copy.copy(self.twarze)
            self.Semaphore_Twarze.release()
            if len(twarze_kopia.obraz) != 0 or not twarze_kopia.obraz.shape:
                faces = twarze_kopia.obraz
                if twarze_kopia.ilosc:
                    for face in twarze_kopia.pozycje:
                        x, y, x1, y1 = [int(i) for i in face]
                        cv2.rectangle(
                            faces, (x, y), (x1, y1),
                            (0, 255, 0), 1
                        )
                        cv2.putText(
                            faces, twarze_kopia.metoda, (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 255, 0), 1
                        )
                    g, g1 = [i for i in twarze_kopia.granice]
                    x, y = [i for i in g]
                    x1, y1 = [i for i in g1]
                    if twarze_kopia.ilosc > 1:
                        cv2.rectangle(
                            faces, (x, y), (x1, y1),
                            (0, 0, 255), 1
                        )
                    cv2.arrowedLine(faces, [int((x + x1) / 2), int((y + y1) / 2)],
                                    (int(faces.shape[1] / 2), int(faces.shape[0] / 2)),
                                    (0, 255, 0), 5)
                batteryLvl = self.Czujniki.battery
                if batteryLvl >= 60:
                    kolor = (0, 255, 0)
                elif batteryLvl >= 35:
                    kolor = (31, 94, 254)
                else:
                    kolor = (0, 0, 255)
                cv2.putText(faces, "Battery " + str(batteryLvl), (0, int(faces.shape[0] - 30)),
                            cv2.FONT_HERSHEY_COMPLEX, 1,
                            kolor, 2)
                if self.metoda_sterowanie == 1:
                    cv2.putText(faces, "PID YAW " + "P" + str(round(self.pid_yaw.Kp, 2)) + " I" + str(
                        round(self.pid_yaw.Ki, 2)) + " D" + str(
                        round(self.pid_yaw.Kd, 2)), (0, int(faces.shape[0] - 60)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor,
                                2)
                    p_yaw, i_yaw, d_yaw = self.pid_yaw.components
                    cv2.putText(faces,
                                "PID YAW " + "P" + str(round(p_yaw, 2)) + " I" + str(round(i_yaw, 2)) + " D" + str(
                                    round(d_yaw, 2)) + " O" + str(round(self.yaw_pid_last_output, 0)),
                                (0, int(faces.shape[0] - 90)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                    cv2.putText(faces, "PID Z " + "P" + str(round(self.pid_z.Kp, 2)) + " I" + str(
                        round(self.pid_z.Ki, 2)) + " D" + str(
                        round(self.pid_z.Kd, 2)), (0, int(faces.shape[0] - 120)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor, 2)
                    p_z, i_z, d_z = self.pid_z.components
                    cv2.putText(faces, "PID Z " + "P" + str(round(p_z, 2)) + " I" + str(round(i_z, 2)) + " D" + str(
                        round(d_z, 2)) + " O" + str(round(self.z_pid_last_output, 0)), (0, int(faces.shape[0] - 150)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                    cv2.putText(faces, "PID X " + "P" + str(round(self.pid_x.Kp, 2)) + " I" + str(
                        round(self.pid_x.Ki, 2)) + " D" + str(
                        round(self.pid_x.Kd, 2)), (0, int(faces.shape[0] - 180)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor, 2)
                    p_x, i_x, d_x = self.pid_x.components
                    cv2.putText(faces, "PID X " + "P" + str(round(p_x, 2)) + " I" + str(round(i_x, 2)) + " D" + str(
                        round(d_x, 2)) + " O" + str(round(self.x_pid_last_output, 0)), (0, int(faces.shape[0] - 210)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                elif self.metoda_sterowanie == 2:
                    cv2.putText(faces, "PID YAW " + "P" + str(round(self.a_pid_yaw.Kp, 2)) + " I" + str(
                        round(self.a_pid_yaw.Ki, 2)) + " D" + str(
                        round(self.a_pid_yaw.Kd, 2)), (0, int(faces.shape[0] - 60)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor,
                                2)
                    cv2.putText(faces,
                                "PID YAW " + " O" + str(round(self.yaw_pid_last_output, 0)),
                                (0, int(faces.shape[0] - 90)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                    cv2.putText(faces, "PID Z " + "P" + str(round(self.pid_z.Kp, 2)) + " I" + str(
                        round(self.pid_z.Ki, 2)) + " D" + str(
                        round(self.pid_z.Kd, 2)), (0, int(faces.shape[0] - 120)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor, 2)
                    cv2.putText(faces, "PID Z " + " O" + str(round(self.z_pid_last_output, 0)),
                                (0, int(faces.shape[0] - 150)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                    cv2.putText(faces, "PID X " + "P" + str(round(self.pid_x.Kp, 2)) + " I" + str(
                        round(self.pid_x.Ki, 2)) + " D" + str(
                        round(self.pid_x.Kd, 2)), (0, int(faces.shape[0] - 180)), cv2.FONT_HERSHEY_COMPLEX, 1, kolor, 2)
                    cv2.putText(faces, "PID X " + " O" + str(round(self.x_pid_last_output, 0)),
                                (0, int(faces.shape[0] - 210)),
                                cv2.FONT_HERSHEY_COMPLEX, 1,
                                kolor, 2)
                if (time.time_ns() - self.Detekcja_rysowanie_time) == 0:
                    cv2.putText(faces, str(int(1e+9 / 1)),
                                (faces.shape[1] - 25, 12),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5,
                                (255, 0, 0), 1)
                else:
                    cv2.putText(faces, str(int(1e+9 / (time.time_ns() - self.Detekcja_rysowanie_time))),
                                (faces.shape[1] - 25, 12),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5,
                                (255, 0, 0), 1)
                self.Detekcja_rysowanie_time = time.time_ns()

                cv2.imshow('Video_detect', faces)
                cv2.waitKey(1)

    dron_w_powietrzu = False
    dron_motoron = False

    def Klawiatura_klawisze(self):
        def k_1():
            self.metoda = 1
            print("Metoda 1 - DSFDDetector")

        def k_2():
            self.metoda = 2
            print("Metoda 2 - Haar cascade")

        def k_3():
            self.metoda = 3
            print("Metoda 3 - mp_face_detection")

        def k_4():
            self.metoda = 4
            print("Metoda 4 - MTCNN")

        def k_5():
            self.metoda = 5
            print("Metoda 5 - RetinaFace")

        def k_6():
            self.metoda = 6
            print("Metoda 6 - Yolo3")

        def k_l():
            while self.dron_w_powietrzu == "inprogress":
                pass
            self.tello_drone.land()
            self.dron_w_powietrzu = False
            self.dron_motoron = False

        def k_t():
            if not self.dron_w_powietrzu:
                self.dron_w_powietrzu = "inprogress"
                self.tello_drone.takeoff()
                self.dron_w_powietrzu = True
                self.dron_motoron = False

        def k_m():
            if self.dron_sdk == '20' or self.dron_sdk == '30':
                if self.dron_motoron:
                    self.tello_drone.turn_motor_off()
                    self.dron_motoron = False
                else:
                    self.tello_drone.turn_motor_on()
                    self.dron_motoron = True

        def k_a():
            self.metoda_sterowanie = 2
            print("Metoda sterowania - Advanced PID")

        def k_z():
            self.metoda_sterowanie = 1
            print("Metoda sterowania - PID")

        keyboard.add_hotkey(2, k_1)
        keyboard.add_hotkey(3, k_2)
        keyboard.add_hotkey(4, k_3)
        keyboard.add_hotkey(5, k_4)
        keyboard.add_hotkey('5', k_5)
        keyboard.add_hotkey('6', k_6)
        keyboard.add_hotkey('t', k_t)
        keyboard.add_hotkey('l', k_l)
        keyboard.add_hotkey('m', k_m)
        keyboard.add_hotkey('a', k_a)
        keyboard.add_hotkey('z', k_z)
        keyboard.wait()

    pid_yaw = PID(0.100, 0.01, 0.005, setpoint=0, sample_time=None)
    pid_yaw.output_limits = (-25, 25)
    pid_yaw.proportional_on_measurement = False

    pid_z = PID(-0.200, -0.02, -0.01, setpoint=0, sample_time=None)
    pid_z.output_limits = (-25, 25)
    pid_z.proportional_on_measurement = False

    x_setpoint = 0.025
    pid_x = PID(100, 10, 100, setpoint=x_setpoint, sample_time=None)
    pid_x.output_limits = (-25, 25)
    pid_x.proportional_on_measurement = False

    yaw_pid_last_output = 0
    z_pid_last_output = 0
    x_pid_last_output = 0

    nazwa_pliku_PID = "Log/" + datetime.now().strftime("%Y-%m-%d %H-%M-%S") + " PID.csv"
    pierwszy_PID = True

    nazwa_pliku_a_PID = "Log/" + datetime.now().strftime("%Y-%m-%d %H-%M-%S") + " a_PID.csv"
    pierwszy_a_PID = True

    ostatnia_zmiana_pid = dt.datetime.now()
    ostatnia_sterowanie = dt.datetime.now()
    zero_rc = False

    Sterowanie_dronem_poprzedni_hash = 0

    metoda_sterowanie = 2

    a_pid_yaw = advanced_pid.PID(Kp=0.100, Ki=0.01, Kd=0.005, Tf=2)
    a_pid_yaw.set_output_limits(-25, 25)

    a_pid_z = advanced_pid.PID(Kp=-0.200, Ki=-0.02, Kd=-0.01, Tf=2)
    a_pid_z.set_output_limits(-25, 25)

    a_pid_x = advanced_pid.PID(Kp=100, Ki=10, Kd=100, Tf=2)
    a_pid_x.set_output_limits(-25, 25)

    def Sterowanie_dronem(self):
        def Zapisz_Naglowek_PID():
            if not os.path.exists("Log"):
                os.mkdir("Log")
            with open(TelloController.nazwa_pliku_PID, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                dane = ['datestamp', 'Yaw output', 'Yaw input', 'Yaw', 'Z output', 'Z input', 'Z', 'X output',
                        'X input', 'Setpoint X', 'Center_X',
                        'Center Y', 'accX',
                        'accY', 'accZ', 'Vx', 'Vy', 'Vz', 'P_yaw', 'I_yaw', 'D_yaw', 'P_yaw_prog', 'I_yaw_prog',
                        'D_yaw_prog', 'P_z', 'I_z', 'D_z', 'P_z_prog', 'I_z_prog', 'D_z_prog', 'P_x', 'I_x', 'D_x',
                        'P_x_prog', 'I_x_prog', 'D_x_prog', 'Dron_w_powietrzu']
                csvwriter.writerow(dane)

        def Zapisz_PID(twarze_kopia):
            with open(TelloController.nazwa_pliku_PID, 'a', newline='') as csvfile:
                csvwriter = csv.writer(csvfile, dialect='unix')  # unix, excel, excel_tab, unix_dialect
                dane = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), yaw,
                        (twarze_kopia.obraz.shape[1] / 2) - twarze_kopia.pozycja_srodka[0],
                        self.Czujniki.yaw, z,
                        (twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1],
                        self.Czujniki.height,
                        x,
                        ((twarze_kopia.granice[1][0] - twarze_kopia.granice[0][0]) * (
                                twarze_kopia.granice[1][1] - twarze_kopia.granice[0][1])) / (
                                twarze_kopia.obraz.shape[0] * twarze_kopia.obraz.shape[1]),
                        self.x_setpoint,
                        twarze_kopia.pozycja_srodka[0],
                        twarze_kopia.pozycja_granice_srodka[1],
                        self.Czujniki.accX, self.Czujniki.accY, self.Czujniki.accZ,
                        self.Czujniki.Vx, self.Czujniki.Vy, self.Czujniki.Vz,
                        p_yaw, i_yaw, d_yaw, self.pid_yaw.Kp, self.pid_yaw.Ki, self.pid_yaw.Kd, p_z, i_z, d_z,
                        self.pid_z.Kp, self.pid_z.Ki, self.pid_z.Kd, p_x, i_x, d_x,
                        self.pid_x.Kp, self.pid_x.Ki, self.pid_x.Kd, self.dron_w_powietrzu
                        ]
                csvwriter.writerow(dane)

        def Zapisz_Naglowek_a_PID():
            if not os.path.exists("Log"):
                os.mkdir("Log")
            with open(TelloController.nazwa_pliku_a_PID, 'w', newline='') as csvfile:
                csvwriter = csv.writer(csvfile)
                dane = ['datestamp', 'Yaw output', 'Yaw input', 'Yaw', 'Z output', 'Z input', 'Z', 'X output',
                        'X input', 'Setpoint X', 'Center_X',
                        'Center Y', 'accX',
                        'accY', 'accZ', 'Vx', 'Vy', 'Vz', 'P_yaw_prog', 'I_yaw_prog',
                        'D_yaw_prog', 'P_z_prog', 'I_z_prog', 'D_z_prog',
                        'P_x_prog', 'I_x_prog', 'D_x_prog', 'Dron_w_powietrzu']
                csvwriter.writerow(dane)

        def Zapisz_a_PID(twarze_kopia):
            with open(TelloController.nazwa_pliku_a_PID, 'a', newline='') as csvfile:
                csvwriter = csv.writer(csvfile, dialect='unix')  # unix, excel, excel_tab, unix_dialect
                dane = [datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f"), yaw,
                        (twarze_kopia.obraz.shape[1] / 2) - twarze_kopia.pozycja_srodka[0],
                        self.Czujniki.yaw, z,
                        (twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1],
                        self.Czujniki.height,
                        x,
                        ((twarze_kopia.granice[1][0] - twarze_kopia.granice[0][0]) * (
                                twarze_kopia.granice[1][1] - twarze_kopia.granice[0][1])) / (
                                twarze_kopia.obraz.shape[0] * twarze_kopia.obraz.shape[1]),
                        self.x_setpoint,
                        twarze_kopia.pozycja_srodka[0],
                        twarze_kopia.pozycja_granice_srodka[1],
                        self.Czujniki.accX, self.Czujniki.accY, self.Czujniki.accZ,
                        self.Czujniki.Vx, self.Czujniki.Vy, self.Czujniki.Vz,
                        self.a_pid_yaw.Kp, self.a_pid_yaw.Ki, self.a_pid_yaw.Kd,
                        self.a_pid_z.Kp, self.a_pid_z.Ki, self.a_pid_z.Kd,
                        self.a_pid_x.Kp, self.a_pid_x.Ki, self.a_pid_x.Kd, self.dron_w_powietrzu
                        ]
                csvwriter.writerow(dane)

        if (xxhash.xxh3_64_digest(self.twarze.obraz) != self.Sterowanie_dronem_poprzedni_hash):
            self.Semaphore_Twarze.acquire()
            twarze_kopia = copy.copy(self.twarze)
            self.Semaphore_Twarze.release()
            self.Sterowanie_dronem_poprzedni_hash = xxhash.xxh3_64_digest(twarze_kopia.obraz)
            if not twarze_kopia.ilosc or not twarze_kopia.obraz.shape or not twarze_kopia.pozycja_granice_srodka.__len__():
                if not self.zero_rc or (dt.datetime.now() - self.ostatnia_sterowanie).total_seconds() >= 10:
                    self.tello_drone.send_rc_control(0, 0, 0, 0)
                    self.zero_rc = True
                    self.ostatnia_sterowanie = dt.datetime.now()
            else:
                if self.metoda_sterowanie == 1:
                    self.zero_rc = False
                    yaw = round(
                        self.pid_yaw((twarze_kopia.obraz.shape[1] / 2) - twarze_kopia.pozycja_granice_srodka[0]), 0)
                    p_yaw, i_yaw, d_yaw = self.pid_yaw.components
                    if abs(d_yaw) > 1000:
                        yaw = self.yaw_pid_last_output
                    self.yaw_pid_last_output = yaw
                    z = round(self.pid_z((twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1]))
                    p_z, i_z, d_z = self.pid_z.components
                    if abs(d_z) > 1000:
                        z = self.z_pid_last_output
                    self.z_pid_last_output = z
                    x = round(self.pid_x(((twarze_kopia.granice[1][0] - twarze_kopia.granice[0][0]) * (
                            twarze_kopia.granice[1][1] - twarze_kopia.granice[0][1])) / (
                                                 twarze_kopia.obraz.shape[0] * twarze_kopia.obraz.shape[1])))
                    p_x, i_x, d_x = self.pid_x.components
                    if abs(d_x) > 1000:
                        x = self.x_pid_last_output
                    self.x_pid_last_output = x
                    print(
                        "yaw " + str(yaw) + " " + str(
                            (twarze_kopia.obraz.shape[1] / 2) - twarze_kopia.pozycja_granice_srodka[0]))
                    print(
                        "z " + str(z) + " " + str(
                            (twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1]))
                    print("x " + str(x) + " " + str(((twarze_kopia.granice[1][0] - twarze_kopia.granice[0][0]) * (
                            twarze_kopia.granice[1][1] - twarze_kopia.granice[0][1])) / (
                                                            twarze_kopia.obraz.shape[0] * twarze_kopia.obraz.shape[1])))
                    self.tello_drone.send_rc_control(0, int(x), int(z), int(yaw))
                    if self.pierwszy_PID:
                        Zapisz_Naglowek_PID()
                        self.pierwszy_PID = False
                    else:
                        Zapisz_PID(twarze_kopia)

                elif self.metoda_sterowanie == 2:
                    self.zero_rc = False
                    yaw = -round(self.a_pid_yaw.integrate(time.time(), (twarze_kopia.obraz.shape[1] / 2) -
                                                          twarze_kopia.pozycja_granice_srodka[0]), 0)
                    z = -round(self.a_pid_z.integrate(time.time(), (
                            (twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1])), 0)
                    x = round(
                        self.a_pid_x.integrate(time.time(),
                                               (self.x_setpoint - (((twarze_kopia.granice[1][0] -
                                                                     twarze_kopia.granice[0][0]) * (
                                                                            twarze_kopia.granice[1][1] -
                                                                            twarze_kopia.granice[0][1])) / (
                                                                           twarze_kopia.obraz.shape[0] *
                                                                           twarze_kopia.obraz.shape[
                                                                               1])))))
                    self.yaw_pid_last_output = yaw
                    self.z_pid_last_output = z
                    self.x_pid_last_output = x
                    print(
                        "yaw " + str(yaw) + " " + str(
                            (twarze_kopia.obraz.shape[1] / 2) - twarze_kopia.pozycja_granice_srodka[0]))
                    print(
                        "z " + str(z) + " " + str(
                            (twarze_kopia.obraz.shape[0] / 2) - twarze_kopia.pozycja_granice_srodka[1]))
                    print("x " + str(x) + " " + str(((twarze_kopia.granice[1][0] - twarze_kopia.granice[0][0]) * (
                            twarze_kopia.granice[1][1] - twarze_kopia.granice[0][1])) / (
                                                            twarze_kopia.obraz.shape[0] * twarze_kopia.obraz.shape[1])))
                    self.tello_drone.send_rc_control(0, int(x), int(z), int(yaw))
                    if self.pierwszy_PID:
                        Zapisz_Naglowek_a_PID()
                        self.pierwszy_a_PID = False
                    else:
                        Zapisz_a_PID(twarze_kopia)

    Obraz_wyswietl_f_time = time.time_ns()

    def Obraz_wyswietl_f(self):
        while True:
            Obraz = copy.copy(self.Obraz)
            if len(Obraz) == 0:
                continue
            if (time.time_ns() - self.Obraz_wyswietl_f_time) == 0:
                cv2.putText(Obraz, str(int(1e+9 / 1)),
                            (Obraz.shape[1] - 35, 12),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5,
                            (255, 0, 0), 1)
            else:
                cv2.putText(Obraz, str(int(1e+9 / (time.time_ns() - self.Obraz_wyswietl_f_time))),
                            (Obraz.shape[1] - 35, 12),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5,
                            (255, 0, 0), 1)
            cv2.putText(Obraz, str(int(self.Obraz_f_fps)),
                        (Obraz.shape[1] - 35, Obraz.shape[0] - 12),
                        cv2.FONT_HERSHEY_COMPLEX, 0.5,
                        (255, 0, 0), 1)
            self.Obraz_wyswietl_f_time = time.time_ns()
            cv2.imshow('Video', Obraz)
            cv2.waitKey(1)

    dron_sdk = None
    dron_serial_number = None
    dron_active = None
    dron_active_snr = None

    def __init__(self):
        self.kill_switch = self.TelloKillSwitch(self)
        self.kill_switch.start()

        self.stop_controller = Event()

        self.tello_drone = tello.Tello()
        self.tello_drone.connect()

        self.dron_sdk = self.tello_drone.query_sdk_version()
        while self.dron_sdk is None:
            pass
        if self.dron_sdk != 'unknown command: sdk?':
            self.dron_serial_number = self.tello_drone.query_serial_number()
            self.dron_active = self.tello_drone.query_active()
            self.tello_drone.enable_mission_pads()
            self.tello_drone.set_mission_pad_detection_direction(2)
            self.dron_active_snr = self.tello_drone.query_wifi_signal_noise_ratio()

        self.BattertyTimer = self.TelloTimer(10, self.stop_controller, self.printBattery)
        self.BattertyTimer.start()

        self.BatteryLow = self.TelloTimer(1, self.stop_controller, self.BatteryLvl)
        self.BatteryLow.start()

        self.ImportTelemetry_thread = self.TelloTimer(0.1, self.stop_controller, self.ImportTelemetry)
        self.ImportTelemetry_thread.start()

        self.SaveToFileHeader()
        self.SaveTocsvFile = self.TelloTimer(0.1, self.stop_controller, self.SaveToFile)
        self.SaveTocsvFile.start()

        # self.ArtificialHorizon = self.TelloTimer(1 / 60, self.stop_controller, self.ArtificialHorizon)
        # self.ArtificialHorizon.start()
        #
        # self.ArtificialHorizon2 = self.TelloTimer(1 / 60, self.stop_controller, self.ArtificialHorizon2)
        # self.ArtificialHorizon2.start()

        self.Obraz_t = self.TelloNoTimer(self.stop_controller, self.Obraz_f)
        self.Obraz_t.start()

        # self.Obraz_wyswietl_t = self.TelloNoTimer(self.stop_controller, self.Obraz_wyswietl_f)
        # self.Obraz_wyswietl_t.start()

        self.Detekcja = self.TelloNoTimer(self.stop_controller, self.Detekcja)
        self.Detekcja.start()

        self.Detekcja_rysowanie_t = self.TelloTimer(1 / 60, self.stop_controller, self.Detekcja_rysowanie)
        self.Detekcja_rysowanie_t.start()

        self.Sterowanie_dronem_t = self.TelloTimer(1 / 120, self.stop_controller, self.Sterowanie_dronem)
        self.Sterowanie_dronem_t.start()

        self.Klawiatura = self.TelloNoTimer(self.stop_controller, self.Klawiatura_klawisze)
        self.Klawiatura.start()

        # self.Keepalive = self.TelloTimer(10, self.stop_controller, self.tello_drone.send_keepalive())
        # self.Keepalive.start()


if __name__ == '__main__':
    if os.name != 'nt':
        if os.geteuid() != 0:
            print('You need a root privileges!')
        else:
            tc = TelloController()
    else:
        tc = TelloController()
