# Jetson: kontynuacja researchu migracji, 2026-09-23

Kontynuacja [HANDOFF-jetson_migracja_1.md](HANDOFF-jetson_migracja_1.md), wykonana bezpośrednio
na Jetsonie. Pierwszy odczyt runtime zakończono około 14:44 czasu Europe/Warsaw;
później przeprowadzono autoryzowane próby na podniesionych kołach. Jazdy po ziemi nie było.

**Stan po 15:49:** VESC/odom działają, geometria lidar/TF jest skorygowana. Próby potwierdziły
ruszanie do przodu, stop RB i skręt od lewego boxa. Naprawiono skok przy wznowieniu oraz
opóźnienia inferencji przez ustawienie jednego wątku CPU.
Zmienna reakcja na prawy box została odtworzona bez ruchu: zmieniające się skany, zwłaszcza
zaniki mapowane na 20 m, zmieniają kierunek pierwszej decyzji modelu. Wdrożono poprawną
obsługę niepoprawnych sąsiadów interpolacji i uzupełnianie krótkich luk w pojedynczym skanie:
replay **150/150 od boxa dla każdej strony**, **37/37 testów**, build OK. AI uruchomiono
ponownie do próby na podniesionych kołach z limitem 0.5; wynik fizyczny jest oczekiwany.
Bringup i pasywne monitorowanie VESC działają. Samodzielna jazda po ziemi pozostaje
niezweryfikowana. Szczegóły aktualizacji są na końcu; pierwotny
audyt poniżej opisuje stan około 14:44 i wcześniejsze usterki.

## Wynik

**Aktywne AI używa `offset=-90`, `direction=-1` i modelu mpo2.** Wskazania kartonów są
zgodne z tym mapowaniem: przód pojazdu jest blisko raw 0°, lewo blisko +90°, prawo blisko
-90°. Wcześniejsze twierdzenie, że raw przód musi być ±180° ze względu na montaż obudowy,
nie zgadza się z zebranymi pomiarami.

**TF nadal ma yaw π.** To obraca raw przód na tył `base_link` i raw lewo na prawo. SAC
nie stosuje tego TF do `/scan`, więc ten sam raw scan może trafiać poprawnie do konwertera
SAC i jednocześnie być błędnie ustawiony względem bazy w SLAM. Zgodnie z pomiarami
kandydatem do późniejszej korekty TF jest yaw 0, ale parametrów i procesów nie zmieniano.
Należy jeszcze potwierdzić oś `base_link` względem fizycznego auta oraz widok w RViz.

**Bringup nie ma działającego VESC ani odometrii.** Sterownik VESC zakończył pracę z powodu
braku `/dev/vesc`. Proces AI żyje, lecz przy zablokowanej autonomii wysyła stop i nie
wykonuje ścieżki inferencji. Brak `/odom` uniemożliwiłby ją również po odblokowaniu.

Wojtek w tej sesji wyraźnie zabronił samodzielnego włączania autonomii i potwierdził, że
box stoi po lewej stronie. Odczyty były pasywne; nie wywoływano enable, nie publikowano
poleceń ruchu, nie restartowano bringupu/AI/panelu i nie wykonywano `colcon build`.

## Maszyna, checkout i uruchomione pliki

- Repo `Beba-ai-ml/ros2_ws2`, `~/ros2_ws`, branch `fix/sim-parity-20260913`.
- Start pracy: HEAD `4a3b1a6`, 20 zmienionych plików śledzonych i nieśledzony
  `tools/ros2_input_diagnostic.py`. Ten ostatni był identyczny z wersją z GitHuba.
- Fetch pobrał handoff `2df1fc0`. Istniejące lokalne zmiany zapisano jako `8ca0698`,
  po czym scalono zdalną gałąź bez konfliktów i bez force push. Bieżący research jest
  kolejnym commitem tej samej gałęzi; `main` pozostaje wcześniejszą wersją.
- Jetson: L4T R35.6.1, ROS2 Foxy, Python 3.8. `key_drive.service`: inactive, disabled.
- Panel PID 5174 działa z **`~/ros2_panel`**, poza repo. Wersja repo jest przenośna;
  aktywna kopia ma ścieżki lokalne. Ich komendy Bringup, AI i SLAM prowadzą do tego samego
  workspace i tych samych konfiguracji. Nie podmieniano działającego panelu.
- Bringup PID 20665, AI launch PID 21020, AI node PID 21153. AI wystartował o 14:02 z
  `--params-file ~/ros2_ws/src/sac_driver/config/driver_params.yaml`.
- Porównanie bajtów source/install: identyczne driver YAML, `sac_driver_node.py`,
  `lidar_converter.py`, launch bringupu, `vesc.yaml` i `joy_mode_manager.py` przed zmianami
  komentarzy w tej sesji. Późniejsze zmiany opisów nie wymagają restartu.
- Wagi source/install mają ten sam SHA256:
  `07e458a6ee797326e79e0e522d3291db2c8bf90a9df397a4d19a5bbc380673a8`.
- Zachowano wcześniejsze lokalne limity VESC: `speed_min/max = ±3525 ERPM`,
  `brake_min/max = ±35000`. To synchronizacja istniejących ustawień, bez nowej kalibracji.

## Potwierdzone odczyty runtime

Parametry odczytano przez `GetParameters`, TF z `/tf_static`, dane przez subskrypcje
RELIABLE depth 10 (TF również TRANSIENT_LOCAL). Nie odczytywano pamięci obiektu konwertera.
Kod tworzy go raz przy starcie i nie aktualizuje przez callback parametrów; wartości
usługi, pliku i czasu jego modyfikacji są tu zgodne, lecz późniejszy `param set` sam w sobie
nie dowodzi przebudowania konwertera.

| Element | Wynik |
|---|---|
| `/sac_driver` offset / direction | `-90.0 / -1.0` |
| `model.path` | `weights/session_Sesja_mpo2_2_policy.pth` |
| tick / decyzja | `60.0 Hz / co 8 ticków` |
| wejścia AI | `/scan`, `/odom`, `/autonomy_lock` |
| `control.enable_on_start` | `false` |
| `/autonomy_lock` | `true`, około 50 Hz, publisher `joy_mode_manager` |
| `/drive` | około 50 Hz, ostatnia próbka speed/steer/accel = 0; logi wskazują stop |
| `/joy` | około 19.75 Hz, ostatnia próbka: wszystkie przyciski 0 |
| `/scan` | 181 wiadomości / około 18 s, około 10.05 Hz, 720 ranges, frame `laser` |
| sterownik lidaru | `inverted=false`, `angle_compensate=true`, `/dev/ttyUSB0`, `Standard` |
| `/scan` publisher | jeden `sllidar_node`, RELIABLE |
| `/odom` | publisher `vesc_to_odom_node` istnieje, lecz 0 wiadomości w oknie pomiaru |
| `/sensors/core` | brak publishera i wiadomości |
| `/commands/servo/position` | publisher istnieje, 0 wiadomości w oknie pomiaru |
| TF `base_link -> laser` | xyz `(0.27, 0, 0.11)`, quaternion `(0, 0, 1, ~0)`, yaw π |
| `/slam_toolbox` | brak działającego procesu i usługi; aktywnych parametrów SLAM nie odczytano |

Około 50 Hz zerowych `/drive` pochodzi z blokady publikowanej co 20 ms: `_on_estop`
zeruje `_last_stop_sent`, więc kolejne ticki ponawiają stop. Nie jest to zmierzona
częstotliwość inferencji ani dowód decyzji sieci.

VESC: `~/.ros/log/vesc_driver_node_20809_1790164931492.log` zawiera błąd otwarcia
`/dev/vesc` (No such file or directory); launch odnotował zakończenie PID 20809 o 14:02:14.
`lsusb` nie pokazał oczekiwanego urządzenia `0483:5740`. Trzeba sprawdzić zasilanie,
USB i enumerację, zanim diagnozuje się odometrię jako błąd SAC. Sama obecność publishera
`/odom` nie oznacza, że płyną dane.

## Przeliczenie pomiarów kartonem

Odczytano lokalnie `log/ros2_input_diag_20260923_134138.jsonl`, pełne zakresy sześciu skanów.
SHA256: `57e164470d9509c6a0b000dc3863533f642ae8e5b1a30fe1791e8c1e2b38f29f`.
Surowy log pozostaje ignorowany przez Git. Konwerter uruchomiono offline dwukrotnie na
tych samych zakresach, z offsetami -90 i +90 oraz kierunkiem -1.

| Etykieta Wojtka | Raw minimum | Indeks / kąt modelu przy -90 | Indeks / kąt modelu przy +90 |
|---|---|---|---|
| bez niczego | -43.8°, 1.074 m | 268 / 134° | 427 / 314° |
| przód | -5.8°, 0.237 m | 192 / 96° | 408 / 276° |
| prawo | -100.9°, 0.166 m | 361 / 182° | 21 / 10.5° |
| tył | +167.5°, 0.224 m | 411 / 282° | 205 / 102.5° |
| lewo | +99.9°, 0.167 m | 6 / 3° | 357 / 178.5° |
| przód | +15.3°, 0.256 m | 149 / 74.5° | 397 / 254° |

Przód modelu jest przy kącie 90° / indeksie 180; lewo przy 0° / indeksie 0,
prawo przy 180° / indeksie 360, tył przy 270°. Minimum nie musi wypaść dokładnie na osi:
box ma szerokość, może stać ukośnie, interpolacja i rzadsze promienie z tyłu zmieniają
wybrany punkt. Przykładowo boczny box może obejmować również tylną półpłaszczyznę.

Świeży monitor pokazał najbliższy obiekt przy raw +89.4° do +105.4°, 0.16–0.17 m.
Wojtek potwierdził w rozmowie, że aktualny box jest po lewej. To dodatkowy punkt zgodności
z zapisanymi pomiarami. Przeliczone promienie są rekonstrukcją diagnostyczną, a nie
odczytem wewnętrznego wektora działającej sieci. Nie oceniano zachowania polityki na torze.

## TF, obudowa skanera i SLAM

Według [definicji LaserScan dla Foxy](https://github.com/ros2/common_interfaces/blob/foxy/sensor_msgs/msg/LaserScan.msg)
kąty odnoszą się do `header.frame_id`, zero jest wzdłuż +X, dodatni obrót wokół +Z.
To konwencja wiadomości. Kierunku opublikowanego skanu nie należy wyprowadzać wyłącznie
z wyglądu lub oznaczeń obudowy.

Lokalny `src/sllidar_ros2/src/sllidar_node.cpp`, funkcja `publish_scan`, przelicza kąty
urządzenia przez `pi - angle` i odwraca tablicę zależnie od `inverted` i kolejności kątów.
Są więc trzy osobne operacje: przeliczenie sterownika do `/scan`, TF pomiędzy ramkami
i przeliczenie raw skanu na promienie sieci.

Aktywna komenda `static_transform_publisher 0.27 0 0.11 pi 0 0 base_link laser` oznacza
yaw π. [Kod programu Foxy](https://github.com/ros2/geometry2/blob/foxy/tf2_ros/src/static_transform_broadcaster_program.cpp)
potwierdza kolejność argumentów `yaw pitch roll`; odebrany quaternion potwierdza wynik.
Nie jest to omyłkowo ustawiony roll.

Panel wskazuje `src/slam_toolbox/config/mapper_params_online_async.yaml`:
`base_frame=base_link`, `odom_frame=odom`, `scan_topic=/scan`. Inny plik z `base_frame=laser`
nie jest wskazywany przez ten przycisk. SLAM nie był uruchomiony, więc nie ma tu dowodu
jego bieżącego działania, parametrów usługi ani poprawności mapy.

## Częstotliwość i opóźnienie

Zapisane skany mają 9.97–10.00 Hz i `scan_time` 98.1–100.3 ms. Świeży monitor miał około
10 Hz, 720 próbek i krok 0.501°. To około 7200 próbek raw na sekundę; wcześniejsze 8 Hz
nie opisuje tej sesji. Konwerter tworzy 450 wartości na wejście obserwacji.

W zapisach wiek nagłówka przy capture wynosi 105.1–111.7 ms; świeży monitor pokazał
109–114 ms przy wieku od odbioru około 10–12 ms. Nagłówek LaserScan oznacza początek
akwizycji, a zebranie obrotu zajmuje około 100 ms. Wiek nagłówka nie jest więc samym
opóźnieniem DDS. W końcowym audycie ostatni dostępny skan miał 193 ms: pomiar był zrobiony
pomiędzy kolejnymi odbiorami. Nie jest to p95 ani opóźnienie od przeszkody do silnika.

Kod zbiera stack przy 60 Hz, lecz nowe skany przychodzą około 10 Hz: kilka ticków powtarza
tę samą geometrię. Polityka ma decyzję co 8 ticków, czyli nominalnie 7.5 Hz. To potencjalna
różnica czasowa wobec symulatora do dalszych pomiarów. Historycznych 5–6 ms inferencji
nie zmierzono ponownie; przy blokadzie i braku odometrii sieć nie jest wywoływana.

## Timeouty parametrów i powtarzalna diagnostyka

Zwykłe `ros2 param get` kończyło się timeoutem, a świeży klient czasem widział tylko
część grafu lub same klienty CLI. `ROS_LOCALHOST_ONLY=1` nie wystarczyło. Osobny profil
UDP-only z jawnymi peerami loopback umożliwił odczyt pełnego grafu, parametrów AI/lidaru
i TF, bez zmiany konfiguracji działających procesów. Zmieniono równocześnie transport
klienta i listę peerów: ten eksperyment nie rozstrzyga, czy przyczyną jest SHM, multicast
czy inny szczegół discovery. Nie usuwano `/dev/shm` ani nie zabijano daemonów.
CLI zwróciło później `Node not found` także z profilem UDP, zarówno przez daemon, jak
i z `--no-daemon --spin-time 3`. Dlatego odczyt bezpośredni czeka na samą usługę przez
ograniczony czas, zamiast wymagać wcześniejszej obecności węzła na liście grafu CLI.

Sprawdzony profil zapisano jako `tools/diagnostics/fastdds_local_readonly.xml`.
Dotyczy domain 0 i portów discovery 7410–7432 znalezionych przez `ss -unap`; nie jest
uniwersalną konfiguracją sieci pojazdu. Użycie tylko dla konkretnego odczytu:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 tools/read_sac_parameters.py --udp-local
```

Nie ustawiaj tego profilu globalnie dla bringupu. Jeżeli przyszłe procesy użyją innych
portów discovery, trzeba ponownie odczytać porty i dostosować listę diagnostyczną.
`read_sac_parameters.py` wywołuje wyłącznie `GetParameters`; nie ustawia parametrów,
nie wywołuje enable, nie publikuje poleceń sterowania. Domyślnie kończy oczekiwanie po 20 s.
Sprawdzono tę komendę na działającym AI: ponownie zwróciła -90/-1, model mpo2,
`enable_on_start=false`, 60 Hz i decyzję co 8 ticków.
Pełny JSON audytu pozostaje lokalnie w `log/jetson_readonly_audit_20260923_144424.json`.

## Sprawdzenia i dalsze kroki

`python3 -m unittest src/sac_driver/test/test_sim_parity.py`: **15 testów, 11 zaliczonych,
4 niezaliczone**. Nie przechodzą front/left/right/rear: istniejące lokalne testy zakładają
raw front ±180°, a YAML ma -90. Nie zmieniano oczekiwań ani kalibracji tylko po to, żeby
uzyskać zielony wynik. Nowy raport zastępuje wcześniejsze nieaktualne stwierdzenia o 15/15.
Składnia zmienionych plików Python, nowego czytnika parametrów i profilu XML jest poprawna;
`git diff --check` nie wykazał problemów. Czytnik parametrów sprawdzono pasywnie na runtime.

1. Ustalić powód braku VESC na USB. Restart bringupu sam nie naprawi brakującego urządzenia.
2. Z autonomią nadal zablokowaną potwierdzić kierunki raw oraz osi `base_link` w RViz;
   uzgodnić TF z pomiarami. Nie przestawiać `base_frame` SLAM na `laser` jako obejścia.
3. Po rozstrzygnięciu geometrii ujednolicić fallbacki Python i założenia testów z YAML.
   Obecnie uruchomienie bez YAML jest inną konfiguracją niż przycisk AI.
4. Zmiana samego YAML/fallbacku wymaga ponownego uruchomienia **AI Inference**. Zmiana
   launch/TF wymaga przebudowania `f1tenth_stack` i restartu **Bringup**. Każdy taki krok
   trzeba skoordynować z Wojtkiem; w tej sesji niczego nie restartowano.
5. Test skrętu/napędu pozostaje osobnym krokiem po zgodzie Wojtka i potwierdzeniu warunków
   wymaganych w `AGENTS.md`. Obecny zakaz autonomii obowiązuje. Nie oznaczać migracji ani
   naprawy zachowania auta jako zakończonej na podstawie samych odczytów.

## Aktualizacja 14:56–15:01: VESC i odometria przywrócone

Log jądra wykazał błąd USB `-71` oraz wielokrotne pojawianie się urządzenia `0483:5740`
i rozłączenia. Ostatnie wykrycie przed stabilnym monitoringiem nastąpiło o 14:55:43.
Nie ustalono, czy przyczyną był kabel, styk, zasilanie czy inny element połączenia.

Od 14:56:07 bezpośredni odczyt `COMM_FW_VERSION` odpowiadał wielokrotnie:
**firmware 6.02, HW 60**. Port `/dev/vesc` wskazywał `/dev/ttyACM0`. Trzy odczyty
`COMM_GET_VALUES` wykazały 11.5 V, 0 ERPM, zerowy prąd silnika/wejścia i fault code 0.
Temperatura FET wynosiła 28.7–28.8°C. Pole temperatury silnika miało około -75°C;
nie traktować tego jako rzeczywistej temperatury silnika — stan czujnika nie jest zweryfikowany.

`python3 tools/vesc/vesc_config_upload.py --check-sig` odczytało zgodne sygnatury:
motor `0x2E43A161`, app `0x1D003A2C`. To zgodność formatu definicji konfiguracji z firmware,
nie porównanie wszystkich wartości nastaw. Nie wgrywano konfiguracji ani firmware.

Wojtek potwierdził „Koła w powietrzu — uruchom sam Bringup”. Zakończono własny monitor,
który otwierał port szeregowy; sprawdzono brak innych driverów i nieaktywny `key_drive.service`.
Uruchomiono istniejący zainstalowany launch:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch3.py
```

Launch wystartował o 15:00:52 (PID 59320), driver VESC PID 59392 zgłosił poprawne połączenie.
AI pozostaje wyłączone. Około 18 sekund audytu dało:

| Sygnał | Wynik |
|---|---|
| `/sensors/core` | 818 wiadomości, około 50.24 Hz, 11.5 V, speed 0, fault 0 |
| `/odom` | 818 wiadomości, około 50.44 Hz, linear.x 0, angular.z 0 |
| `/scan` | 182 wiadomości, około 10.05 Hz |
| `/autonomy_lock` | około 50 Hz, `true` |
| `/drive` / `/sac_driver` | brak wiadomości / brak działającego węzła |
| VESC port i limity | `/dev/vesc`, ±3525 ERPM |
| odometria | wheelbase 0.35, `use_servo_cmd_to_calc_angular_velocity=true`, TF włączony |

Okno subskrypcji zaczęło się po komendach neutralnych emitowanych przez startup bringupu,
więc brak odebranych komend serwa w tym oknie nie oznacza, że nie wysłano ich na starcie.
Pełny wynik pozostaje lokalnie w `log/bringup_recovery_audit_20260923.json`.
Bringup działa poza panelem; nie wolno uruchomić drugiego drivera na tym samym porcie.
Monitor USB po starcie bringupu sprawdza wyłącznie obecność urządzenia i nie otwiera portu.

## Nowy pomiar przodu z AI wyłączonym

Po przywróceniu bringupu Wojtek przestawił box dokładnie przed auto i potwierdził gotowość.
Zapisano dwa świeże skany po potwierdzeniu; ostatni trafił jako `front_confirmed` do
`log/lidar_cardboard_recheck_20260923.jsonl`. Minimum raw: **+4.256°, 0.316 m**.
Konwerter -90/-1 wskazał indeks **171**, kąt modelu **85.5°**, odległość **0.316 m**,
czyli blisko osi przodu modelu 90° / indeksu 180.

Ten sam punkt przy obecnej translacji lidaru x=0.27 m ma współrzędne w `base_link`:

| TF | x [m] | y [m] |
|---|---:|---:|
| Obecny yaw π | -0.0451 | -0.0235 |
| Kandydat yaw 0 | +0.5851 | +0.0235 |

Wcześniejszy TF umieszczał potwierdzony fizyczny przód za początkiem układu `base_link`.
Na tym etapie był to rachunek z rzeczywistego skanu, bez zmiany TF. Następnie wykonano
osobno potwierdzone pomiary lewej i prawej strony, opisane niżej.

## Pomiary boczne, korekta i weryfikacja po restarcie 15:13

Wojtek osobno potwierdził przestawienie boxa na lewo i na prawo. Po każdym potwierdzeniu
capture czekał na dwa świeże skany. W tym samym lokalnym JSONL są teraz trzy rekordy:

| Pozycja potwierdzona przez Wojtka | Raw minimum | Indeks modelu przy -90/-1 | Indeks przy +90/-1 |
|---|---|---:|---:|
| przód | +4.256°, 0.316 m | 171 | 399 |
| lewo | +122.921°, 0.281 m | 433 | 291 |
| prawo | -66.843°, 0.318 m | 313 | 438 |

Box po lewej stał częściowo za środkiem lidaru; prawy był nieco przed nim. Nie wymagamy,
żeby ich minima trafiły dokładnie na indeksy 0 i 360. Pozycje w bazie liczone z yaw 0
to odpowiednio `(0.585, +0.023)`, `(0.117, +0.236)`, `(0.395, -0.292)` metra.
Yaw π zamieniał oba znaki boczne i umieszczał przedni punkt przy x=-0.045 m.
Wyniki są spójne z wcześniejszymi sześcioma zapisami oraz konwencją LaserScan.

Na tej podstawie zmieniono:

- `bringup_launch3.py`: yaw π → **0**, bez zmiany translacji;
- `LidarConverter` i fallback `SACDriverNode`: +90 → **-90**, zgodnie z istniejącym YAML;
- cztery testy geometrii: fizyczny raw przód 0, lewo +90, prawo -90, tył ±180;
- dodano regresję dla konwertera uruchomionego bez jawnego offsetu;
- dokumentację i instrukcje agentów, żeby nie przywracały obalonego założenia o raw yaw π.

YAML offset -90, direction -1, speed/steer signs i kalibracja VESC pozostały bez zmian.
Nie zmieniano plików SLAM, limitów prędkości ani logiki deadmana. Nie uruchamiano AI.

`python3 -m unittest src/sac_driver/test/test_sim_parity.py`: **16/16 OK**.
Po zatrzymaniu własnego bringupu i sprawdzeniu, że wszystkie jego dzieci zakończyły pracę,
`colcon build --packages-select sac_driver f1tenth_stack`: **oba pakiety zbudowane poprawnie**.
Bringup ponownie wystartował o 15:13:10, PID 67623; VESC driver PID 67703.

Odczyt po restarcie (`log/lidar_frame_corrected_audit_20260923.json`, około 18 sekund):

| Sprawdzenie | Wynik |
|---|---|
| `/tf_static`, `base_link -> laser` | xyz `(0.27, 0, 0.11)`, quaternion **(0, 0, 0, 1)** |
| `/sensors/core` | 901 wiadomości, **50.02 Hz**, **11.4 V**, fault **0**, ERPM **0** |
| `/odom` | 852 wiadomości, **50.29 Hz**, prędkość zerowa |
| `/scan` | 182 wiadomości, **10.05 Hz** |
| `/autonomy_lock` | **true**, około 50 Hz |
| AI | brak węzła `/sac_driver`, brak wiadomości `/drive` |

USB pozostało obecne podczas planowanego restartu. Pasywny monitor ROS zgłasza przerwy
w danych podczas zatrzymanego bringupu; nie należy ich mylić z rozłączeniem USB.
Bringup jest uruchomiony z terminala diagnostycznego, poza panelem. Monitorowanie USB
i tematów ROS nie otwiera portu szeregowego i nie publikuje komend sterujących.

Zamknięte: zgodność zmierzonej geometrii raw skanu, TF i fallbacków z aktywnym YAML.
Otwarte: fizyczna reakcja skrętu, jazda po ziemi, pomiar pełnego opóźnienia i jakość mapy
SLAM. Pomiary pasywne i testy offline nie zastępują tych prób. Autonomia nadal zabroniona.

## Przygotowanie próby na podniesionych kołach, 15:26

Wojtek ponownie potwierdził podniesione koła i zgodził się na testowanie. Wybrano wariant,
w którym sam przytrzymuje i puszcza RB, a diagnostyka rejestruje sygnały. Ta nowa zgoda
rozszerza wcześniejsze ograniczenie do samego Bringupu na próbę stanowiskową.

Pierwszy start AI z głównym YAML i argumentami `-p control.speed_limit_mps:=0.5` oraz
`-p control.safe_speed_limit_mps:=0.5` nie zastosował limitów: bezpośredni GetParameters
zwrócił **2.0 / 2.0**. Zatrzymano ten proces przed próbą. Nie ustalono jeszcze przyczyny
pierwszeństwa parametrów; przyszłe uruchomienia wymagają odczytu wartości aktywnych.
Ta instancja nadała pojedynczy stop `disabled`, lecz nie pokazała obsługi aktywnej blokady.

Kolejny start używa kopii pełnego YAML w `/tmp/sac_stand_trial_05.yaml`, z obydwoma
limitami ustawionymi na **0.5**. Dla samego procesu AI zastosowano osobny tymczasowy
profil UDP z jawnymi peerami loopback 7410–7472. Wrapper importuje zainstalowany
`SACDriverNode` bez zmian logiki sterowania i co 2 sekundy raportuje jego stan.
Potwierdzono wewnątrz węzła: model załadowany, świeże scan/odom, `enabled=false`,
`lock=true`, efektywny limit **0.5**. Osobny GetParameters potwierdził oba limity,
`safe_mode=true`, `enable_on_start=false`, watchdog **0.5 s**, offset **-90**, direction
**-1**, speed sign **-1**, steer sign **+1**. Główny YAML i Bringup nie zostały zmienione.
To obejście transportu na potrzeby próby, a nie ustalona diagnoza problemu DDS.

Recorder subskrybuje `/joy`, `/autonomy_lock`, `/drive`, `/ackermann_cmd`,
`/teleop_gated`, `/commands/motor/speed`, `/commands/servo/position`, `/sensors/core`,
`/odom` i `/scan`. Zapis: `log/stand_trial_20260923_152504.jsonl` (ignorowany przez git).
Przed próbą: około 11.3 V, fault 0, 0 ERPM, telemetry/odom około 50 Hz, scan 10 Hz.
Poproszono o RB przez około sekundę i puszczenie, przy puszczonym LB, oraz obserwację
fizycznego kierunku kół i zatrzymania. Wynik próby jest jeszcze niepotwierdzony.

### Wynik pierwszego RB: 15:27:36.762–15:27:40.830

Wojtek potwierdził fizyczne ruszanie kół do przodu i zatrzymanie po puszczeniu RB.
Odebrana blokada była zdjęta przez **4.068 s**. Zapis zawiera 168 komend AI i tyle samo
komend wyjściowych muxa: prędkość **-0.005…-0.5 m/s**, silnik **-9.25…-925 ERPM**.
VESC przez cały przedział raportował fault **0**. Odometria miała dodatni znak przy
ujemnym ERPM, zgodnie z potwierdzonym ruchem naprzód na tym aucie.

Od odbioru `/autonomy_lock=true` do pierwszej komendy zerowej minęło:

| Temat | Różnica czasów odbioru |
|---|---:|
| `/ackermann_cmd` | 2.70 ms |
| `/commands/motor/speed` | 3.23 ms |
| `/drive` | 12.95 ms |
| pierwsze `/sensors/core` z ERPM=0 | 350.95 ms |

To czasy odbioru u subskrybenta, nie bezpośredni pomiar opóźnienia od fizycznego przycisku.
Po pierwszym ERPM=0 wystąpiły jeszcze małe odczyty i krótkie odchylenie do -249 ERPM;
pełnego zatrzymania mechanicznego nie wyznaczamy z pojedynczej próbki zerowej.
Szczyt podczas rozpędzania wyniósł **-1412 ERPM**, czyli **0.763 m/s** w odometrii.
Limit 0.5 ograniczył komendy; nie ograniczył tego chwilowego przekroczenia prędkości
nieobciążonych kół. Nie zmieniano konfiguracji regulatora VESC.

Skręt obejmował **-0.346…+0.328 rad** i wielokrotnie zmieniał znak przy stojącym aucie.
Średnia częstość komend około **41 Hz**, mediana odstępu **15.6 ms**, maksimum **207 ms**.
Przyczyna długich odstępów i zmiennego skrętu wymaga pomiaru; nie przypisujemy jej jeszcze
samemu modelowi, CPU ani DDS. Poproszono o box z przodu po lewej i zapis świeżego skanu.
Test start/stop jest zaliczony; omijanie przeszkody i jazda po ziemi pozostają niezweryfikowane.

## Próby skrętu, poprawka wznowienia i liczby wątków CPU

Nowe skany przy puszczonym RB: box z przodu po lewej **+53.8° / 0.47 m** (ray 72),
box z przodu po prawej **-58.3° / 0.49 m** (ray 296). Pełne skany i konwersje w
`log/stand_cardboard_20260923.jsonl`. Wojtek potwierdził, że przy lewym boxie przednie
koła skręciły w prawo. Komendy miały głównie ujemny skręt, więc ten test potwierdza
fizyczny znak tej strony sterowania bez zmiany gainu/offsetu serwa.

### Wznowienie po RB

Dwie kolejne próby zaczynały od razu od -0.5 m/s i niemal pełnego skrętu. W kodzie
`_last_control_time` pochodził z poprzedniego aktywnego ticka. Pierwszy nowy tick liczył
`dt` obejmujące cały postój. Reset mappera nie usuwał tego odstępu. Zmieniono:

- `_publish_stop`: czyści czas ostatniego ticka także przed deduplikacją stopu;
- `_on_timer`: przy `_needs_reset` używa domyślnego dt zamiast czasu od poprzedniej sesji.

Cztery regresje nie tworzą węzła ROS ani publishera: callback jest wywoływany na obiekcie
z atrapami zegara, wejść i publishera. Sprawdzają wznowienie po 170 s, normalny tick,
czyszczenie czasu na stopie i stop deduplikowany. Zmieniony proces zbudowano i uruchomiono
ponownie z limitem 0.5. Pierwsze komendy kolejnych prób wynosiły **-0.005 m/s**.

### Liczba wątków

Przy wyłączonym AI odtworzono ten sam zapisany lewy skan i ten sam stan początkowy;
po 5 rozgrzewkach wykonano 40 decyzji dla każdej liczby wątków:

| Wątki PyTorch intra-op | Mediana | p95 | Maksimum |
|---|---:|---:|---:|
| 6 (poprzedni domyślny stan) | 53.36 ms | 141.16 ms | 160.79 ms |
| 1 | 4.24 ms | 7.20 ms | 21.70 ms |
| 2 | 4.22 ms | 49.55 ms | 129.78 ms |

Akcje zgadzały się z dokładnością około 4e-7. Dodano `model.cpu_threads: 1`, przekazywane
do `InferenceEngine`; pula jest ustawiana przed wczytaniem modelu. Dwie regresje sprawdzają
ustawienie i odrzucenie niepoprawnej liczby. Łącznie **22/22 testy OK**, build `sac_driver` OK.
Weryfikacja aktywnych parametrów: threads 1, limity 0.5/0.5, safe mode true, watchdog 0.5 s.

88 zapisanych decyzji live po zmianie: mediana **4.84 ms**, p95 **10.50 ms**, maksimum
**16.13 ms**. Częstość komend wzrosła do około 59–60 Hz. Pojedyncze dłuższe odstępy odbioru
wciąż występowały, więc nie jest to gwarancja braku jittera całego toru.

### Prawy box i dalsza diagnostyka

Wojtek opisał początek w prawo lub prosto, z przejściem w lewo po około 300 ms.
Nie uznano tego za powtarzalnie zaliczony test omijania. Przy puszczonym RB i ERPM=0
zatrzymano AI (PID 81488). Bringup i pasywne monitory pozostały uruchomione.

Odtworzenie konkretnych zapisanych skanów z zerowymi wejściami ruchu daje pierwszą akcję
skrętu **-0.7645** dla lewego i **+0.8613** dla prawego boxa. Pierwsze akcje live dla
prawego boxa różniły się nawet przy speed=0, servo=0.5, accel=0 i yaw=0. Poproszono o
nieruchomą scenę z odsuniętym obserwatorem, żeby zapisać kolejne skany i zbadać same
pierwsze decyzje modelu bez publikowania ruchu. Źródło zmienności nie jest jeszcze ustalone.

## Nieruchoma scena: 150 skanów i analiza zanikających odczytów, 15:40

Po osobnym potwierdzeniu Wojtka (box po prawej, obserwator odsunięty) zapisano 150 skanów
w 15.1 s. Każdy był podawany do modelu jako nowy stan początkowy: cztery kopie jednej
obserwacji, speed=0, servo=0.5, accel=0, yaw=0. Węzeł diagnostyczny miał tylko subskrypcję
`/scan`; nie sterował pojazdem. Wyniki oraz wszystkie 450-elementowe wejścia zapisano
lokalnie w `log/stationary_policy_probe_20260923.json`.

Pierwsza akcja skrętu obejmowała **-0.99999…+0.99130**. Przy progu ±0.05: **65 w prawo,
8 blisko zera, 77 w lewo**. Ten sam zamrożony skan odtworzony 30 razy dawał dokładnie
identyczną akcję -0.2914221. Zmienność jest więc odtwarzalna od zmiany wejściowych skanów,
bez zmiany odometrii, pada, stanu silnika czy losowania akcji.

Raw skany miały **28–62** wartości niepoprawne lub poza zakresem. Po konwersji **37–86**
promieni modelu wskazywało maksymalne 20 m, z czego **28–76** w przedniej półsferze.
Na kierunkach boxa, indeksy 296 i 297 (raw -58° i -58.5°), odległość około **0.485 m**
przełączała się na **20 m w 18% skanów**. Inne niestabilne promienie, np. raw -36°,
przełączały się z około 0.739 m do 20 m w 28% skanów.

W obecnym konwerterze interpolacja z niefinitywnym sąsiadem też może dać niefinitywny
wynik, który zostaje zastąpiony max range. Ten pomiar nie rozdziela jeszcze wpływu
fizycznego zaniku zwrotów od wzmocnienia jego efektu przez interpolację.

Wyłącznie offline wykonano następujące przekształcenia tych samych 150 wejść:

| Wariant wejścia | W prawo | Prawie prosto | W lewo, od boxa |
|---|---:|---:|---:|
| oryginał | 65 | 8 | 77 |
| mediana per promień z maks. 3 ostatnich skanów | 16 | 1 | 133 |
| mediana per promień z maks. 5 ostatnich skanów | 2 | 0 | 148 |
| max range zastąpiony medianą poprawnych odczytów z całego zapisu | 0 | 0 | 150 |

Ostatni wariant korzysta z przyszłych próbek i służy **tylko do diagnozy**, nie jest
implementacją filtra online. Pierwsze próbki median czasowych miały krótszą historię.
Wyniki w `log/stationary_policy_counterfactual_20260923.json` wskazują, że zaniki do max
range mają duży udział w zmianach decyzji w tej scenie. Nie wdrożono żadnego wygładzania:
należy sprawdzić zachowanie przy pojawieniu/zniknięciu przeszkody i opóźnienie reakcji
przed kolejną próbą na aucie. Samo poprawienie statystyki nieruchomego boxa nie waliduje
jazdy. Nie zmieniano znaków sterowania ani geometrii lidaru.

Stan końcowy tej serii: AI wyłączone; Bringup 67623 i jeden driver VESC 67703 działają;
lock true, ERPM 0, fault 0, około 11.2 V. Pasywne monitory USB i zdrowia ROS działają.
Zamknięto szczegółowy recorder prób; zapisane pliki `*_latest.json` są odtąd historyczne.
Git zawiera poprawki czasu wznowienia, liczby wątków CPU, testy i dokumentację; dane surowe
pozostają w ignorowanym `log/`. Domyślne limity YAML to nadal 2.0 m/s — 0.5 m/s obowiązywało
wyłącznie w tymczasowej konfiguracji prób. Główna gałąź nie jest scalana automatycznie.

## Naprawa brakujących promieni bez historii skanów, 15:49

Na prośbę o kolejny krok zebrano dwie świeże serie po **150 pełnych raw skanów**.
Wojtek osobno potwierdził nieruchomy box po prawej i po lewej; AI było wyłączone.
Zapisy: `log/raw_scan_series_right_20260923.json` i `log/raw_scan_series_left_20260923.json`.
W starszych dwóch pełnych skanach większość braków była pojedynczymi promieniami;
driver SLLIDAR koduje zerowy odczyt jako `inf`. W konwerterze interpolacja
`finite + inf`, a nawet `0 * inf` przy dokładnym indeksie, niszczyła poprawny zwrot.

Porównano te same wejścia i model przy zerowych danych ruchu, resetując stack dla każdego
skanu. Liczba pierwszych decyzji **od boxa**:

| Wariant | Box po prawej: skręt w lewo | Box po lewej: skręt w prawo |
|---|---:|---:|
| poprzedni kod | 74/150 | 132/150 |
| zachowanie poprawnego końca interpolacji | 150/150 | 149/150 |
| dodatkowo krótkie ograniczone luki do 1.5° | 150/150 | 150/150 |

Zaimplementowano ostatni wariant:

1. Przed interpolacją odrzucane są `NaN`, `inf`, wartości <=0 oraz poza `range_min/max`
   czujnika. Przy jednym poprawnym sąsiedzie używany jest ten poprawny pomiar.
2. `lidar.max_invalid_gap_deg: 1.5` ogranicza szerokość uzupełnianej luki, liczoną jako
   liczba brakujących promieni razy krok kątowy. Luka musi mieć poprawne pomiary po obu
   stronach; używana jest bliższa z tych odległości. Wartość 0 wyłącza ten etap.
3. Wszystko dotyczy **jednego bieżącego skanu**. Nie ma mediany czasowej, pamięci poprzedniej
   sceny ani oczekiwania na kilka skanów. Długie i nieograniczone luki pozostają nieuzupełnione;
   gdy oba końce interpolacji są niedostępne, nadal obowiązuje fallback max range.
4. Całkowicie pusty/niepoprawny skan zgłasza błąd, który callback sterowania zamienia na
   komendę stop. Stop resetuje teraz także stan epizodu, żeby powrót danych nie kontynuował
   integratora i historii obserwacji sprzed błędu.

**37/37 testów OK**: wcześniejsze testy oraz przypadki błędnych sąsiadów, zerowej wagi,
krótkich/długich i nieograniczonych luk, nagłego pojawienia/zniknięcia przeszkody,
pustych skanów i komendy zerowej przy wyjątku konwertera. Testy callbacka korzystają
z atrap i nie tworzą węzła/publishera ROS. Build `sac_driver` poprawny.

Osobno odtworzono pełne 300 skanów przez faktyczną implementację: tablice wejścia zgadzają
się z kandydatem bitowo (maksymalna różnica **0**). Akcje lewego boxa **-0.874…-0.571**,
prawego **+0.890…+0.961**. Wyniki w `log/lidar_implemented_replay_20260923.json`; porównanie
wariantów w `log/lidar_candidate_comparison_20260923.json`. To wynik dla zarejestrowanych
scen, nie gwarancja zachowania na dowolnym torze.

Na istniejącej zgodzie na próby z podniesionymi kołami uruchomiono poprawione AI (PID 90859)
z tymczasowym limitem **0.5/0.5 m/s**, cpu_threads **1**, gap **1.5**, safe mode true,
watchdog **0.5 s**. Wszystkie wartości potwierdzono przez GetParameters. Bringup i jedyny
VESC driver nie były restartowane. Recorder zapisuje
`log/stand_trial_20260923_154832.jsonl`, inferencję `log/stand_lidar_fix_inference_20260923.jsonl`.
Poproszono o dwie krótkie próby RB przy lewym boxie; fizyczny wynik jest jeszcze oczekiwany.
