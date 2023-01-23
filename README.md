# Dron
Proponujemy na nasz projekt grupowy z przedmiotu Identyfikacji i sterowania robotów latających zagadnienie programowania drona DJI Tello, który będzie miał za zadanie identyfikacje człowieka i podążanie jego śladem.

Autorzy
* Krzysztof Truszkiewicz
* Adam Dutkiewicz
* Kamil Janiak
* Bartłomiej Sobczak

Zakres\
Zadaniem robota latającego jest przesłanie obrazu z kamery do jednostki
obliczeniowej, która na podstawie parametrów robota rozpozna twarz oraz
określi współrzędne jej na obrazie. Na podstawie tych danych jednostka
latająca będzie dążyć do wycentrowania obrazu kamery na twarz oraz
podążania za znalezioną twarzą.
* Odczyt danych z kamery
* Rozpoznanie twarzy
* Zastosowanie regulatora do sterowania jednostką latającą
* Ruch robota latającego
* Wyświetlanie obrazu
* Zaznaczanie twarzy, jeżeli została wykryta

Działanie programu rozpoczyna się od naawiązaniakontaktu z dronem DJI Tello i określenie jego modelu.\
Następnie uruchamiane jest kilka wątków:
* TelloKillSwitch - odpowiada za awaryjne zatrzymanie drona
* printBattery - wyświetla w terminalu co 10s stan naładowania akumulatora drona
* BatteryLvl - wyświetla w terminalu ostarzenie o niskim poziomie naładowania akumulatora drona
* ImportTelemetry - pobiera co 0.1s dane telemetryczne z drona
* SaveToFile - zapisuje co 0.1s dane telemetryczne z drona
* ArtificialHorizon
* ArtificialHorizon2
* Obraz_f
* Obraz_wyswietl_f
* Detekcja
* Detekcja_rysowanie
* Sterowanie_dronem
* Klawiatura_klawisze
