# Jetson: kontynuacja researchu migracji, 2026-09-23

Kontynuacja [HANDOFF-jetson_migracja_1.md](HANDOFF-jetson_migracja_1.md), wykonana bezpośrednio
na Jetsonie. Odczyt runtime zakończony około 14:44 czasu Europe/Warsaw. To raport z
działających procesów, kodu i istniejących pomiarów; próby jazdy nie wykonano.

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
