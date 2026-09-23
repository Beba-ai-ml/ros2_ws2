# Handoff: jetson_migracja_1

Stan wiedzy: 2026-09-23. Ten plik jest przekazaniem kontekstu dla Codexa uruchomionego
bezpośrednio na Jetsonie. Przeczytaj go przed dalszą diagnozą. Celem jest ustalić, co dokładnie
widzi SAC na wejściu lidarowym i jak raw `/scan`, transform TF, SLAM oraz fizyczne kierunki auta
mają się do siebie. Nie zmieniaj parametrów ani nie ruszaj pojazdem na podstawie samego handoffu.
Fakty o kodzie pochodzą z lokalnego checkoutu repo, a pomiary live z wyników wklejonych przez
Wojtka. Ta sesja nie miała bezpośredniego dostępu do ROS uruchomionego na Jetsonie ani do
podanej ścieżki z jego logiem.

## Cel Wojtka i objawy

- W symulacji AI zachowuje się dobrze, na rzeczywistym aucie jeździ dziwnie, między innymi na boki.
- Wojtek podejrzewa obrót montażu lidaru i pamięta, że ustawiał ujemny offset, aby odwrócić skan.
- Lidar ma być fizycznie zamontowany tyłem, według Wojtka. Nie mamy niezależnego potwierdzenia
  tego montażu ani pomiaru yaw samego urządzenia.
- Wojtek chce, żeby następny Codex działał na Jetsonie i testował na bieżącym ROS. Przy każdym
  kroku wymagającym restartu ma jasno dostać informację, co restartować i polecenie do wykonania.
- Użytkownik kojarzy, że w SLAM zmieniał `base_link` i `scan`/subscriber, i tylko konfiguracja
  używana obecnie przyciskiem SLAM w `ros2_control_panel` działała. Nie zmieniaj jej w ciemno.
- Był przywołany film o rozjeździe kierunku bazy i skanu jako analogia. Nie traktuj filmu ani
  pamięci o nim jako dowodu o konfiguracji tego auta.

## Repozytorium i wersje

- Repo: `Beba-ai-ml/ros2_ws2`, katalog na Jetsonie zwykle `~/ros2_ws`.
- Lokalny checkout, z którego zebrano poniższe fakty: branch
  `fix/sim-parity-20260913`, commit `fbe6df0` (`tools: capture ROS inputs on Enter`), czysty
  przed rozpoczęciem tego handoffu. Zawiera wcześniejsze commity narzędzia diagnostycznego.
- To **nie dowodzi**, że Jetson ma tę samą gałąź, commit, źródła, `install/` ani uruchomiony
  kod. Zacznij od sprawdzenia ich bez modyfikowania.
- Bieżące źródłowe `src/sac_driver/config/driver_params.yaml` w tym branchu ma
  `lidar.angle_offset_deg: -90.0` i `lidar.angle_direction: -1.0`.
- Jetsonowy runtime nie jest jeszcze jednoznacznie potwierdzony. Wcześniejsze
  `ros2 param get /sac_driver lidar.angle_offset_deg` zwróciło `90.0`, a
  `ros2 param get /sac_driver lidar.angle_direction` zwróciło `-1.0`. Odczyt
  `ros2 param get /sac_driver model.path` zakończył się timeoutem usługi.
- Później Wojtek podał z pobranego zdjęcia `-90` i `-1`, a wcześniej wkleił też tekst z Jetsona
  wskazujący `90.0` i `-1.0`. Nie jest jasne, który odczyt dotyczył aktywnych parametrów węzła,
  który pliku, ani czy po odczycie nastąpiła zmiana. Zrób świeży odczyt parametrów po ustaleniu,
  że właściwy `/sac_driver` działa. Nie rozstrzygaj konfliktu na podstawie pamięci.

## Co kod robi z lidarem

### Bringup i fizyczny frame

- Panelowy przycisk Bringup buduje `f1tenth_stack`, source'uje `install/setup.bash` i odpala
  `ros2 launch f1tenth_stack bringup_launch3.py`.
- `bringup_launch3.py` odpala `sllidar_node` z `frame_id=laser`, `inverted=false`,
  `angle_compensate=true`, domyślnie `/dev/ttyUSB0`, 256000 baud.
- Ten launch publikuje statyczny TF `base_link -> laser` z translacją `(0.27, 0, 0.11)` i yaw
  `0.0`. Repo nie deklaruje tu obrotu fizycznego skanera o 180 stopni.
- W `bringup_launch3.py` został komentarz o „Rotated scan republisher (yaw +30 deg)”, ale
  `scan_yaw_rotator` nie występuje na liście nodów tego launchu. Sam komentarz nie oznacza,
  że raw `/scan` jest obracany. Node `scan_yaw_rotator.py` i ścieżka `/scan_2` są osobnym,
  starszym kodem.
- Parametr `inverted` sterownika trzeba odróżnić od yaw TF i od mapowania sieci. Wartość z pliku
  nie zastępuje sprawdzenia rzeczywistych `LaserScan.angle_min`, `angle_increment`, `frame_id`
  i pomiaru przeszkody względem pojazdu.

### AI i indeksy promieni

- Przycisk AI Inference w `ros2_panel/process_manager.py` buduje `sac_driver`, ładuje
  `install/setup.bash` i odpala `sac_driver_node` z parametrami z
  `src/sac_driver/config/driver_params.yaml`.
- `sac_driver` subskrybuje `/scan` i wewnętrznie wywołuje `LidarConverter`. Nie publikuje
  osobnego, obróconego topicu skanu dla SLAM.
- Do każdej klatki stanu konwerter podaje 450 promieni lidarowych; pełna obserwacja modelu jest
  składana z 4 klatek i ma 1820 wartości. Raw skan RPLIDAR-a w pomiarze miał 720 próbek.
  **Indeks 180 to indeks w wektorze sieci, nie 180 stopni.** Przy obecnym generatorze kątów
  indeks 0 odpowiada modelowemu kątowi 0 stopni, indeks 180 odpowiada 90 stopni, czyli przodowi,
  a indeks 360 odpowiada 180 stopni. Dalsze promienie opisują tylną półpłaszczyznę. Nie ma tu
  720-stopniowego układu.
- Modelowy kąt `a` jest mapowany do kąta skanu wzorem
  `scan_angle = wrap(angle_direction * (a + angle_offset_deg))`.
  Para `offset=-90`, `direction=-1` daje `scan_angle = 90 - a`, czyli zamierzoną zgodność
  symulatora z ROS: przód symulatora (`a=90`) trafia na przód ROS (`0`). To jest zamierzona
  konwencja w obecnym kodzie; nadal trzeba zweryfikować, czy odpowiada fizycznemu autu.
- Węzeł tworzy `LidarConverter` raz, podczas inicjalizacji. W kodzie nie znaleziono
  `add_on_set_parameters_callback`. Samo `ros2 param set` nie jest dowodem, że istniejący obiekt
  konwertera zmienił mapowanie. Po zmianie YAML parametr ma być wczytany przez ponowne uruchomienie
  procesu AI.

### SLAM z przycisku panelu

- `ros2_panel/process_manager.py`, przycisk SLAM (process id 2), uruchamia RViz i
  `slam_toolbox online_async_launch.py` z plikiem
  `src/slam_toolbox/config/mapper_params_online_async.yaml`.
- Ten konkretny plik ma `base_frame: base_link` i `scan_topic: /scan`.
- Jest też `src/f1tenth_stack/config/f1tenth_online_async.yaml` z `base_frame: laser`, ale
  aktualna komenda przycisku panelu go nie wskazuje.
- SLAM czyta oryginalny `/scan`; offset SAC nie obraca wejścia SLAM. Dla SLAM istotne są
  `LaserScan.header.frame_id` oraz poprawny TF pomiędzy `laser`, `base_link` i `odom`.
  Najpierw potwierdź aktywne parametry `/slam_toolbox`, źródło jego procesu i TF. Nie przełączaj
  `base_frame` między `laser` i `base_link` bez dowodu z działającej sesji i RViz.

## Dane z kartonów, podane przez Wojtka

Diagnostyka została odpalona w trybie capture. Jej nagłówek podał, że dla obliczenia AI-ray używa
`rays=450`, `offset=+90.0`, `direction=-1.0`, `max_range=20.0m` jako nadpisania CLI. Zapis miał
trafić do:
`/home/laptop/ros2_ws/log/ros2_input_diag_20260923_134138.jsonl`.

| Etykieta ustawiona przez użytkownika | Najbliższa przeszkoda w raw `/scan` | Najbliższy promień po lokalnym konwerterze diagnostycznym |
|---|---:|---:|
| bez niczego | `-43.8 deg`, `1.07 m` | `427`, `1.08 m` |
| przód | `-5.8 deg`, `0.24 m` | `408`, `0.24 m` |
| prawo | `-100.9 deg`, `0.17 m` | `21`, `0.17 m` |
| tył | `+167.5 deg`, `0.22 m` | `205`, `0.22 m` |
| lewo | `+99.9 deg`, `0.17 m` | `357`, `0.17 m` |
| przód | `+15.3 deg`, `0.26 m` | `397`, `0.26 m` |

Interpretacja i ograniczenia:

- Podane etykiety z przodu skupiają się w raw `/scan` koło zera, prawa strona jest ujemna,
  lewa dodatnia, tył jest koło 180 stopni. To sugeruje, że opisane kierunki raw skanu są bliskie
  kierunkom pojazdu. Pomiar zależy jednak od tego, gdzie dokładnie Wojtek ustawił karton; nie
  potwierdza samodzielnie fizycznego yaw ani TF.
- AI-ray w tabeli został policzony przez osobny skrypt diagnostyczny, który sam zastosował
  `+90/-1`. **To nie jest odczyt wektora z działającego procesu sieci ani dowód jego aktywnej
  konfiguracji.** Dla `+90/-1` front raw trafia w okolice indeksów 397-408 zamiast oczekiwanego
  indeksu 180. To pasuje do obrócenia mapowania względem konwencji `-90/-1`, ale trzeba to
  potwierdzić na żywym runtime i aktualnym pliku.
- Narzędzie zapisuje pełne zakresy, odometrię, drive i serwo do JSONL, ale tylko odczytuje topic'i.
  Nie publikuje komend. Log z Jetsona może być niedostępny w checkoutie PC. Nie commituj surowego
  logu z pojazdu do repo; odczytaj go lokalnie, jeśli jest potrzebny.
- Wojtek pytał o opóźnienie przy ok. 720 próbkach i 8 Hz. To daje `5760` próbek raw na sekundę
  z podanej specyfikacji, a konwerter wytwarza 450 promieni na scan. `.context/STATE.md` zapisuje
  historyczny czas inference około 5-6 ms na Jetson CPU, ale nie jest to pomiar bieżącego
  opóźnienia wejścia. Sprawdź timestampy i wiek scanów z live diagnostyki, zanim uznasz latency
  za przyczynę.

## Jak kontynuować na Jetsonie

1. Potwierdź maszynę i checkout przed komendami ROS: `hostname`, `pwd -P`, branch, commit,
   `git status --short`; sprawdź, czy `~/ros2_ws` jest repo opisanym powyżej i czy commit zawiera
   `tools/ros2_input_diagnostic.py`.
2. Przeczytaj `AGENTS.md`, `.context/INDEX.md`, `.context/STATE.md`,
   `.context/HANDOFF-20260913-sim-parity.md` i ten plik.
3. Najpierw tylko odczyt: sprawdź `key_drive.service`, uruchomione nody, kto publikuje `/scan`,
   subskrypcje `/sac_driver`, aktualne parametry `lidar.angle_offset_deg`,
   `lidar.angle_direction`, `model.path`, `/slam_toolbox` i TF. Poprzedni timeout `model.path`
   wyjaśnij przez sprawdzenie węzła/usługi, nie przez zgadywanie.
4. Porównaj świeży odczyt live z bieżącym YAML i sprawdź, czy node AI uruchomił dokładnie
   checkout, który myślisz. Nie zmieniaj offsetu, kierunku skanu, TF, `steer_sign` ani kalibracji
   VESC na podstawie samego wyniku diagnostycznego.
5. Jeśli trzeba zmienić wyłącznie `driver_params.yaml`, wyjaśnij Wojtkowi, że restartuje się
   **AI Inference**, nie Bringup i nie cały panel. Plik jest wczytywany przy starcie procesu.
   Nie restartuj procesu, jeśli ktokolwiek może właśnie prowadzić lub testować.
6. Przed testem wymagającym ruchu najpierw wykonaj pomiary read-only z zatrzymanym pojazdem,
   RB zwolnionym i autonomią zablokowaną. Żadnych publikacji na `/drive`, `/teleop_gated`,
   `/commands/motor/*` ani enable AI. Testy kierunku wykonuj tylko po wyraźnym potwierdzeniu
   Wojtka, że koła są w górze albo auto jest na pustym torze. Sprawdź zasady `AGENTS.md` dotyczące
   pojedynczego procesu VESC i `key_drive.service` przed uruchamianiem lub zatrzymywaniem nody.
7. Gdy poprosisz Wojtka o wykonanie kroku, podaj jedną konkretną komendę/akcję, napisz co ma
   zostać uruchomione lub zrestartowane, a potem poproś o pełny wynik. Nie każ mu zgadywać, co
   oznacza „restartuj ROS”.

Nie uznawaj pojazdu za naprawiony, dopóki nie zostaną pogodzone aktywne parametry z YAML,
geometria raw `/scan`, indeksy wejścia SAC, TF używany przez SLAM i fizyczny test lewej oraz
prawej strony. Wcześniejszy offline test sim-parity jest testem kodu, a nie weryfikacją na aucie.
