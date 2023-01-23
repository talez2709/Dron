# Dron
Proponujemy na nasz projekt grupowy z przedmiotu Identyfikacji i sterowania robotów latających zagadnienie programowania drona DJI Tello, który będzie miał za zadanie identyfikacje człowieka i podążanie jego śladem.

Autorzy
* Krzysztof Truszkiewicz
* Adam Dutkiewicz
* Kamil Janiak
* Bartłomiej Sobczak

Zakres\
Zadaniem robota latającego jest przesłanie obrazu z kamery do jednostki obliczeniowej, która na podstawie parametrów robota rozpozna twarz oraz określi współrzędne jej na obrazie. Na podstawie tych danych jednostka latająca będzie dążyć do wycentrowania obrazu kamery na twarz oraz podążania za znalezioną twarzą.
* Odczyt danych z kamery
* Rozpoznanie twarzy
* Zastosowanie regulatora do sterowania jednostką latającą
* Ruch robota latającego
* Wyświetlanie obrazu
* Zaznaczanie twarzy, jeżeli została wykryta

Działanie programu rozpoczyna się od naawiązania kontaktu z dronem DJI Tello i określenie jego modelu.\
Następnie uruchamiane jest kilka wątków:
* TelloKillSwitch - odpowiada za awaryjne zatrzymanie drona po naciśnięciu spacji
* printBattery - wyświetla w terminalu co 10s stan naładowania akumulatora drona
* BatteryLvl - wyświetla w terminalu ostarzenie o niskim poziomie naładowania akumulatora drona
* ImportTelemetry - pobiera co 0.1s dane telemetryczne z drona
* SaveToFile - zapisuje co 0.1s dane telemetryczne z drona
* ArtificialHorizon - wyświetla sztuczny horyzont z naniesiony na obraz z kamery drona
* ArtificialHorizon2 - wyświetla sztuczny horyzont podobny do rzeczywistego urządzenia
* Obraz_f- pobiera obraz z kamery drona
* Obraz_wyswietl_f - wyświetla obraz z kamery drona
* Detekcja - wykonuje detekcji twarzy na obrazie z kamery drona
* Detekcja_rysowanie - wykonuje narysowanie ramki wokoło twarzy
* Sterowanie_dronem - oblicza i wysyła polecenia sterowania algorytmem PID dronem celem ustawienie środka obrazu z kamery drona środkiem do twarzy
* Klawiatura_klawisze - wykrywa naciśnięcie klawiszy celem sterowania algorytmem
  * Wybór algorytmu detekcji twarzy
    * 1 DSFDDetector
    * 2 Haar cascade
    * 3 mp_face_detection
    * 4 MTCNN
    * 5 RetinaFace
    * 6 Yolo3
  * Sterowanie silnikami drona
    * t Wystarowanie drona
    * l wylądowanie dronem
    * m włączonie silników na wolne obroty celem chłodzenia drona - działa tylko dla DJI RoboMaster TT Tello Talent 
  * Wybór algorytmu sterowania dronem
    * a Advanced PID - PID z filterm filtr pochodnej pierwszego rzędu z stalą czasową 2s (czas przesłania obrazu z drona do programu).
    * z PID
 
Program po przez działania w wątkach pozwala na asynchroniczne przetwarzanie obrazu redukuje to opóźnienia w sterowaniu.


Przykład działania\
![](https://github.com/talez2709/Dron/blob/main/demo.gif)


