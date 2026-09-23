# Handoff na 2026-09-24: autonomia i pozostałe testy

**Stan końcowy: 2026-09-23 około 16:01, Europe/Warsaw.**
Wojtek poprosił o zapis wszystkich wyników, problemów i planu na jutro oraz push na
GitHub. Po zamknięciu naszych procesów o 15:55 poprosił jeszcze o AI, ale następnie
**wycofał tę prośbę: tylko dokończyć zapisywanie, nie włączać autonomii**.
Asystent nie uruchomił ponownie AI/Bringupu po tej prośbie. Zastane nowe procesy panelu
też zniknęły przed końcowym sprawdzeniem o 16:01. **Nie uruchamiać niczego ponownie dzisiaj.**

## 1. Od czego zacząć jutro

1. Przeczytać ten plik, [STATE.md](STATE.md) i `AGENTS.md`.
2. Repo: `Beba-ai-ml/ros2_ws2`, branch **`fix/sim-parity-20260913`**, workspace `~/ros2_ws`.
   Nie pracować przypadkiem na starszym `main`; tej gałęzi nie scalono do `main`.
3. Ostatni kod: **0a0562a** — naprawa brakujących promieni lidaru.
   Wcześniej **7bb55d2** — czas wznowienia i liczba wątków CPU,
   **b76ad37** — TF i fallbacki lidaru. Późniejszy commit dokumentacji zamyka dzień.
4. **Test na podniesionych kołach zaliczony przez użytkownika:** ruch do przodu,
   zatrzymanie po puszczeniu RB, box po lewej -> skręt w prawo, box po prawej -> skręt
   w lewo. Użytkownik napisał, że działają oba kierunki.
5. **Jazdy po ziemi nie zaliczono ani nie wykonano w naszych testach.** Przed nią potrzebne są aktualne potwierdzenie wolnego
   miejsca i ustawienie małego limitu; nie włączać autonomii samowolnie na podstawie
   wczorajszej zgody na podniesione koła.
6. **Główny YAML nadal ma 2.0 m/s.** Dzisiejsze próby miały **0.5/0.5 m/s** z osobnego
   pliku. Zwykłe kliknięcie AI w panelu nie odtwarza tych próbnych limitów.

## 2. Stan urządzenia na koniec dnia

- Testowe AI zatrzymano przy RB puszczonym i ERPM=0.
- Około 15:55 zatrzymano również własny Bringup, wszystkie jego dzieci oraz monitory
  diagnostyczne. Sprawdzenie PID-ów potwierdziło zakończenie procesów — brak osieroconego
  VESC, joy, muxa, lidaru i statycznego TF z tej sesji.
- `key_drive.service`: **inactive, disabled**. Nie włączać go podczas prób.
- **Nie ma monitorowania VESC w tle na noc.** Alarm USB był częścią zakończonej sesji.
- Przed zatrzymaniem: `/autonomy_lock=true`, ERPM=0, fault=0, około **11.1 V**.
  To historyczny pomiar, nie jutrzejszy stan akumulatora.
- Source i install były zgodne po udanym buildzie `sac_driver`; wcześniej zbudowano też
  `f1tenth_stack`. Nie trzeba przebudowywać bez powodu przed ustaleniem aktualnego stanu.
- Asystent nie wyłączał komputera ani GUI panelu. Panel działał z **`~/ros2_panel`**, poza repo;
  nasze dzisiejsze Bringup/AI działały z terminala, nie z jego kart.

### Krótka zmiana stanu po zamknięciu sesji

Około 15:57 panel uruchomił nowy Bringup (96060, VESC 96247) i AI (96551, parent 96422),
co wykryto pasywnie przy późniejszej prośbie użytkownika. Odczyt aktywnych parametrów
potwierdził główny YAML **2.0/2.0**, a nie przygotowany limit 0.5. Użytkownika poinformowano.
Lock był true, RB puszczone, drive=0, ERPM=0, fault=0. W tym świeżym uruchomieniu AI
odbierało lock przy domyślnym DDS — problem komunikacji jest więc zależny od sytuacji.

Wojtek potwierdził wtedy, że auto jest na ziemi i miejsce jest wolne, lecz zaraz potem
**odwołał prośbę włączenia autonomii**. To ostatnie polecenie ma pierwszeństwo. Asystent
przygotował wyłącznie pliki `log/driver_params_test_05.yaml` i `log/fastdds_stand_trial_udp.xml`;
nie zastosował ich do procesu panelu, nie wywołał enable i nie wysłał komend ruchu.
O 16:00:56 i w następnym sprawdzeniu żaden z tych nowych PID-ów już nie działał.
Zamknięto również nowy recorder pasywny 97933. Ostatecznie **AI/Bringup i monitory sesji
są zatrzymane**; jutrzejszy agent musi ponownie sprawdzić faktyczny stan.

## 3. Wszystkie istotne problemy i ich status

| Problem / objaw | Co ustalono i zrobiono | Status na jutro |
|---|---|---|
| VESC znikał z USB, brak `/dev/vesc`, brak `/sensors/core` i `/odom` | W kernelu rozłączenia i błąd `-71`; użytkownik pracował przy połączeniu. VESC wrócił około 14:55–14:56, odpowiadał FW 6.02 / HW 60. Potem odometria i telemetria około 50 Hz, fault 0. | Działał do końca prób. Przyczyny sprzętowej nie udowodniono; ponownie sprawdzić USB i stabilność. |
| AI było uruchomione, ale nie wykonywało normalnej inferencji | Było zablokowane, a brak VESC powodował brak odometrii. Sam żywy proces nie oznacza gotowości sterowania. | Po przywróceniu danych i właściwej komunikacji inferencja działała. Zawsze sprawdzać wejścia i lock. |
| Niezgodne założenia o kierunku lidaru | YAML/live -90/-1, fallback Python +90, TF yaw π; część testów zakładała przeciwny raw frame. Pomiary boxa ustaliły raw przód około 0°, lewo dodatnie, prawo ujemne. | TF yaw **0**, fallback **-90**, direction **-1**; 16 testów geometrii/parity przechodzi. Nie przywracać starego +90/π. |
| Skręt raz ku boxowi, raz od boxa przy nieruchomej scenie | Brakujący promień `inf` psuł interpolację z poprawnym sąsiadem. Box około 0.49 m trafiał do sieci jako 20 m; 150 skanów dawało 65 decyzji w prawo i 77 w lewo. | Naprawione zachowanie poprawnego sąsiada oraz krótkie luki do 1.5° w jednym skanie. Replay 150/150 od boxa na każdą stronę; użytkownik potwierdził obie strony na aucie. |
| Pusty/całkowicie błędny skan wyglądał jak wolna przestrzeń | Poprzednio pusty skan zwracał wektor max range. | Teraz wyjątek konwertera -> stop; sprawdzone w teście callbacka z atrapami. Fizyczny test utraty danych pozostaje do wykonania. |
| Po ponownym RB skok od razu do 0.5 m/s | `dt` obejmowało cały czas z puszczonym RB. Reset mappera nie zerował czasu ostatniego aktywnego ticka. | Stop czyści zegar i stan epizodu; wznowienie używa domyślnego dt. Start po postoju około -0.005 m/s; potwierdzone testami i zapisem auta. |
| Długie obliczenia sieci, komendy około 41–47 Hz | PyTorch korzystał z 6 wątków. Na tym samym stanie mediana 53.36 ms, p95 141.16 ms. | `model.cpu_threads=1`; benchmark mediana 4.24 ms, p95 7.20 ms. Końcowe próby około 58–60 Hz. Nadal są pojedyncze dłuższe odstępy. |
| DDS/ROS CLI nie widziało węzłów lub parametry timeoutowały | Domyślne discovery zawodziło. Pierwsze AI z domyślnym transportem nadało stop, ale nie pokazywało odbioru lock. Izolowany profil UDP z jawnymi peerami loopback umożliwił odczyty i właściwe wejścia AI. | **Obejście, nie rozwiązana przyczyna.** Nie uznawać zwykłego startu z panelu za zweryfikowany. Profile stosowano per proces, nie globalnie. |
| Nadpisanie limitu przez CLI `-p` nie zadziałało | Po starcie z głównym YAML i `-p ...:=0.5` GetParameters wciąż zwracał **2.0/2.0**. Proces zatrzymano przed próbą. | Pełna kopia YAML z 0.5/0.5 zadziałała. Zawsze odczytać parametry aktywne. Przyczyna pierwszeństwa parametrów pozostaje otwarta. |
| Przekroczenie mierzonej prędkości mimo limitu 0.5 | Komenda była ograniczona do 925 ERPM, ale nieobciążone koła chwilowo osiągały do około 1432 ERPM, czyli 0.774 m/s w odometrii. | Nie zmieniano PID ani kalibracji VESC. Limit komendy nie jest twardym limitem chwilowej prędkości; sprawdzić reakcję pod obciążeniem. |
| Pomiar czasu stopu łatwo źle zinterpretować | Czasy po lock są czasami odbioru tematów. Pierwsza próbka ERPM=0 nie oznacza pełnego uspokojenia; część RB ponownie wciskano przed ustaniem obrotów. | Nie deklarować zmierzonego czasu fizyczny przycisk -> pełny stop. Potrzebny osobny pomiar. |
| Ręczny pad ma inny limit niż AI | `joy_teleop.yaml` ma skalę prędkości -5.0; limit 0.5 AI go nie ogranicza. Przy zamykaniu odczytano starsze logi około 15:22 z komendami -9250 ERPM obcinanymi przez VESC do -3525. Nie zapisano wtedy źródła tych komend; wielkość odpowiada konfiguracji teleop. | Przed ręczną próbą po ziemi osobno sprawdzić/ograniczyć teleop. Nie przypisywać tych starszych komend do późniejszych prób AI z 0.5. |
| Błędy podczas zamykania Bringupu | Przy SIGINT `joy_mode_manager` wypisał KeyboardInterrupt / exit -2, a `joy_linux_node` zakończył się RCLError o nieaktywnym kontekście / exit -6. VESC, lidar, mux i pozostałe węzły zakończyły pracę. | Zapisane jako problem zamykania; po sprawdzeniu brak osieroconych procesów. Nie mylić z awarią w trakcie jazdy. |
| Panel i procesy z terminala mogą się rozjechać | Aktywny panel jest poza repo, a sesja diagnostyczna uruchamiała procesy samodzielnie. | Liczyć rzeczywiste procesy. **Dokładnie jeden driver `/dev/vesc`**, nie uruchamiać drugiego Bringupu. |

Dodatkowo odczyt temperatury silnika około -75°C z surowego VESC nie został uznany za
rzeczywistą temperaturę — czujnik/konfiguracja tego pomiaru są niezweryfikowane. Temperatura
FET przy postoju była około 29°C. Nie wgrywano konfiguracji motor/app; odczytane sygnatury
pasowały do definicji firmware 6.02.

## 4. Ważne ustawienia, których nie zgubić

| Ustawienie | Wartość |
|---|---|
| Model | `weights/session_Sesja_mpo2_2_policy.pth`, policy-only, CPU, torch 1.13 |
| Model / stan | 450 promieni, 455 wartości na obserwację, stack 4 = 1820 |
| Wątki CPU | `model.cpu_threads=1` |
| Sterowanie | tick 60 Hz, decyzja co 8 ticków |
| Lidar | offset -90°, direction -1, `max_invalid_gap_deg=1.5`, max range modelu 20 m |
| TF base_link -> laser | xyz `(0.27, 0, 0.11)`, yaw 0 |
| Znaki | `speed_sign=-1`, `steer_sign=+1`; dodatnie drive.speed oznacza cofanie tego auta |
| Prędkość próbna | **oba** `control.speed_limit_mps` i `control.safe_speed_limit_mps` = 0.5 |
| Prędkość w głównym YAML | **oba nadal 2.0** |
| Bezpieczeństwo | safe mode true, enable_on_start false, watchdog 0.5 s |
| Pad | RB = AI podczas trzymania; puszczenie = blokada/zero; LB = manual i priorytet nad AI |
| Mux | AI `/drive` priorytet 10, `/teleop_gated` priorytet 100, lock `/autonomy_lock` |
| Rzeczywiste wyjście muxa | `/ackermann_cmd` (nie zakładać, że nazwa z nieskutecznego remapu w launchu jest tematem wyjścia) |
| VESC | gain 1850 ERPM/(m/s), limity ±3525 ERPM |
| Serwo | gain -0.9, offset 0.5304, limity 0.05/0.95 |
| Odometria | wheelbase 0.35, yaw wyliczany z komendy serwa, nie z niezależnego IMU |

Nie zmieniano znaków, gainów ani limitów kalibracji VESC. Model ma akcję przyspieszenia
w zakresie **[0,2]**; nie traktować go jako gwarantowanego hamulca przed boxem. Zatrzymanie
RB/muxa jest osobnym mechanizmem.

Naprawa lidaru jest bez historii: poprawny sąsiad zostaje zachowany; krótka luka ograniczona
poprawnymi promieniami jest wypełniana bliższą odległością. Długie/nieograniczone luki nie
są wypełniane; dwa niedostępne końce interpolacji nadal dają max range. Nie wdrożono mediany
czasowej z 3/5 skanów — badano ją tylko offline.

## 5. Co rzeczywiście sprawdzono

- Surowe firmware/telemetria VESC po odzyskaniu USB, potem jeden działający driver.
- Scan około 10 Hz, telemetry/odom około 50 Hz; TF po korekcie quaternion `(0,0,0,1)`.
- Trzy osobno potwierdzone pozycje boxa do ustalenia geometrii.
- Dwie serie po 150 pełnych raw skanów do oceny naprawy zanikających promieni.
- Faktyczna implementacja daje te same tablice co oceniony kandydat (maks. różnica 0).
- **37/37 testów**: 16 parity, 14 obsługi błędnych promieni, 5 czasu/stopu sterowania,
  2 konfiguracji wątków. Build `sac_driver` poprawny. Testy callbacka używają atrap i nie
  publikują do sprzętu.
- Fizyczne próby z podniesionymi kołami, limitem 0.5 i deadmanem użytkownika:
  przód, stop RB, obie strony skrętu. Ostatnie siedem okien RB: fault 0 i komendy w limitach.
- Po naprawie: 318 decyzji, mediana 4.37 ms, p95 9.60 ms, max 33.26 ms. Komenda motor=0
  po odebranym lock=true w 0.89–17.89 ms. To nie pomiar pełnej latencji mechanicznej.

## 6. Kolejność jutrzejszych prób

### A. Start i krótka kontrola na podniesionych kołach

1. Potwierdzić aktualne warunki fizyczne przed uruchomieniem Bringupu/komend.
2. Sprawdzić `key_drive.service`, procesy, `/dev/vesc`, `/dev/rplidar`, pad i bieżące
   napięcie. Jeżeli USB znów znika, najpierw wrócić do stabilności połączenia.
3. Uruchomić jeden Bringup. Sprawdzić świeże scan/odom/telemetrię, fault 0, puszczony RB
   i lock=true. Nie traktować starych plików `*_latest.json` jako świeżego stanu.
4. Przygotować **pełny testowy YAML** z obydwoma limitami 0.5. Nie polegać na `-p` ani na
   zmianie parametrów w już uruchomionym węźle: obiekty sterowania powstają na starcie.
5. Uruchomić jedno AI z przygotowanym YAML, odczytać aktywne parametry przez
   `python3 tools/read_sac_parameters.py --udp-local`. Jeśli transport domyślny znów nie
   działa, odtworzyć dzisiejszy profil UDP per proces; nie eksportować go globalnie.
6. Krótko powtórzyć RB start/stop i box lewo/prawo. Nie zwiększać prędkości na podstawie
   samego udanego startu procesu. Przed kolejnym impulsem pozwolić kołom się zatrzymać.

### B. Pozostałe sprawdzenia przed jazdą

- Priorytet LB nad RB, ze świadomie ustawioną małą prędkością teleop i neutralnymi osiami.
- Kontrolowana utrata/starość wejść scan/odom i ich powrót: stop, świeży stan przy wznowieniu.
  Najpierw test diagnostyczny bez ruchu lub na podniesionych kołach; nie zaczynać od zaniku
  danych w trakcie jazdy po ziemi.
- Zatrzymanie i ponowny start procesu AI/Bringupu, sprzątnięcie dzieci, brak drugiego drivera.
- Sprawdzić czy skan z naprawą pokazuje nową przeszkodę w bieżącej ramce i usuwa ją po
  przestawieniu; testy syntetyczne to potwierdzają, scenę fizyczną sprawdzić osobno.
- Osobno przeanalizować pozostałe skoki czasów odbioru/obliczeń oraz odpowiedź regulatora
  prędkości pod obciążeniem. Nie zmieniać kalibracji bez pomiaru i ponownej próby.

### C. Pierwsza próba na ziemi

- Dopiero po aktualnym potwierdzeniu przez Wojtka wolnego miejsca i gotowości.
- Pozostać przy **0.5 m/s**, RB w ręku użytkownika, krótkie odcinki i najpierw stop.
- Ocenić kierunek jazdy, stabilność skrętu, zachowanie na realnym ruchu, opóźnienie stopu,
  stabilność USB i świeżość danych. Nie traktować boxa jako dowodu automatycznego hamowania.
- Dopiero z nowymi wynikami zdecydować o dalszym zakresie prób/prędkości.

### D. Pozostała migracja / SLAM

- SLAM/RViz i jakość mapy nie były testowane. Panel używa apt `slam_toolbox` oraz
  `src/slam_toolbox/config/mapper_params_online_async.yaml`, base_link/odom i `/scan`.
  Dzisiaj konfiguracji SLAM nie zmieniano. Sprawdzić ustawienie skanu po TF yaw 0 i jakość
  mapowania w osobnej próbie, bez dublowania Bringupu.
- Doprowadzić do powtarzalnego startu z panelu z właściwymi limitami i działającym DDS.
  Samą synchronizacją repo nie podmieniono aktywnego `~/ros2_panel`.

## 7. Odtworzenie ustawień próbnych

Poniższy fragment **tylko zapisuje pliki**, nie uruchamia ROS ani ruchu. Uruchamiać z
roota workspace. Tworzy pełny YAML z małym limitem i odtwarza użyty profil UDP domain 0
z peerami loopback 7410–7472. To obejście diagnostyczne z dzisiejszej sesji.

```bash
cd ~/ros2_ws
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

out = Path('log')
out.mkdir(exist_ok=True)
data = yaml.safe_load(Path('src/sac_driver/config/driver_params.yaml').read_text())
p = data['sac_driver']['ros__parameters']
p['control.speed_limit_mps'] = 0.5
p['control.safe_speed_limit_mps'] = 0.5
p['control.enable_on_start'] = False
(out / 'driver_params_test_05.yaml').write_text(yaml.safe_dump(data, sort_keys=False))

ns = 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'
ET.register_namespace('', ns)
tree = ET.parse('tools/diagnostics/fastdds_local_readonly.xml')
peers = tree.find('.//{%s}initialPeersList' % ns)
existing = {int(e.text) for e in peers.findall('.//{%s}port' % ns)}
for port in range(7410, 7474, 2):
    if port not in existing:
        locator = ET.SubElement(peers, '{%s}locator' % ns)
        udp = ET.SubElement(locator, '{%s}udpv4' % ns)
        ET.SubElement(udp, '{%s}address' % ns).text = '127.0.0.1'
        ET.SubElement(udp, '{%s}port' % ns).text = str(port)
tree.write(str(out / 'fastdds_stand_trial_udp.xml'), encoding='utf-8', xml_declaration=True)
PY
```

**Dopiero po wykonaniu preflight i potwierdzeniu warunków fizycznych**: zwykły
`ros2 run sac_driver sac_driver_node` z `--params-file "$PWD/log/driver_params_test_05.yaml"`.
W razie potrzeby zastosować dla tego procesu
`FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/log/fastdds_stand_trial_udp.xml"`.
Najpierw źródłować `/opt/ros/foxy/setup.bash` i `install/setup.bash`. Nie uruchamiać AI,
gdy inna instancja już istnieje. Nie wywoływać enable i nie publikować sztucznego lock/joy
zamiast sprawdzenia rzeczywistego deadmana.

## 8. Gdzie są wszystkie dane i szczegóły

- Pełna chronologia: [RESEARCH-jetson-20260923.md](RESEARCH-jetson-20260923.md).
- Poprzedni handoff: [HANDOFF-jetson_migracja_1.md](HANDOFF-jetson_migracja_1.md).
- Stan i kolejne zmiany: [STATE.md](STATE.md).
- W repo są kod, konfiguracja, testy i dokumentacja z wynikami. **Surowe logi są zachowane
  lokalnie w ignorowanym `log/`**, zgodnie z zasadami repo; sam clone ich nie pobierze.

| Plik lokalny w `log/` | Zawartość |
|---|---|
| `jetson_readonly_audit_20260923_144424.json` | Pierwszy audyt, brak VESC/odom |
| `bringup_recovery_audit_20260923.json` | Odczyt po odzyskaniu VESC |
| `lidar_frame_corrected_audit_20260923.json` | Live TF i dane po korekcie |
| `lidar_cardboard_recheck_20260923.jsonl` | Potwierdzone front/left/right do geometrii |
| `stand_cardboard_20260923.jsonl` | Dwa pełne skany ukośnych boxów przed próbami |
| `stand_trial_20260923_152504.jsonl` | Komendy/telemetria pierwszych prób |
| `stand_trial_inference_20260923.jsonl` | Decyzje po zmianie CPU, przed naprawą lidaru |
| `stationary_policy_probe_20260923.json` | 150 wejść nieruchomej sceny, zmienny skręt |
| `stationary_policy_counterfactual_20260923.json` | Porównanie mediany i eksperymentu z max range |
| `raw_scan_series_left_20260923.json`, `raw_scan_series_right_20260923.json` | Pełne raw skany 150+150 |
| `lidar_candidate_comparison_20260923.json` | Porównanie wariantów naprawy |
| `lidar_implemented_replay_20260923.json` | Wynik rzeczywistej implementacji, 150/150 na stronę |
| `stand_trial_20260923_154832.jsonl` | Końcowe próby po naprawie lidaru |
| `stand_trial_20260923_155933.jsonl` | Pasywny odczyt późniejszego startu z panelu; nie próba inicjowana przez asystenta |
| `stand_lidar_fix_inference_20260923.jsonl` | 318 decyzji i czasy obliczeń |
| `vesc_usb_watch_20260923.log` | Historia obecności USB |
| `session_20260923_support/` | 14 zarchiwizowanych skryptów/profili/YAML z `/tmp` |
| `session_20260923_support/MANIFEST.json` | Rozmiary i SHA256 plików pomocniczych |

Archiwum skryptów jest zapisem eksperymentu: część ma ścieżki tego Jetsona i historyczne
nazwy logów. Nie uruchamiać wszystkich plików w ciemno, szczególnie wrappera tworzącego AI.
Profile i testowy YAML można odtworzyć z sekcji 7 bez zależności od `/tmp`.
Pliki `stand_trial_latest.json` / `stand_trial_ai_latest.json` to **zamknięte migawki**.

Dzisiejsze znane niepewności: przyczyna USB i DDS, zachowanie pod obciążeniem, pełna
latencja mechaniczna, reszta jittera, fizyczne testy utraty danych, SLAM i jazda po ziemi.
Nie zastępować ich stwierdzeniem „wszystko gotowe do autonomii”.
