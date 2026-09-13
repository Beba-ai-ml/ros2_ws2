# Review jazdy autonomicznej (sac_driver) - dlaczego auto „robi dziwne rzeczy" (13.09.2026)

Zakres: tylko ścieżka AI (lidar → stan → sieć → /drive). Nic nie naprawiono - to lista błędów z dowodami
dla sesji naprawczej. Źródła: `~/ros2_ws2` (main `6f08f38`, = GitHub) i trening `~/occupancy_racer/Soft_Actor_Critic_2`
(HEAD `5b59934` = GitHub `occupancy-racer-sac2` master, zero różnic w kodzie; starszy o 8 commitów jest tylko
klon w `Soft_Actor_Critic_3/`).

Jak auto rusza (z `ros2_panel/process_manager.py`): przycisk **Bringup** = `colcon build f1tenth_stack` +
`ros2 launch f1tenth_stack bringup_launch3.py` (joy, joy_teleop, joy_mode_manager, vesc_driver, ackermann_to_vesc,
vesc_to_odom, ackermann_mux, static tf, sllidar S1; throttle_interpolator WYŁĄCZONY). Przycisk **AI Inference** =
`colcon build sac_driver` + `ros2 run sac_driver sac_driver_node --params-file src/sac_driver/config/driver_params.yaml`
(NIE launch file; model = `weights/session_Rybnik_02_1.pth`). Pad: LB (4) = teleop deadman, RB (5) = autonomia
(`joy_mode_manager.py:48-52` publikuje `autonomy_lock=false` tylko gdy RB trzymany i LB puszczony; sac_driver
na `lock=false` sam się włącza, `sac_driver_node.py` `_on_estop`). Mux: `teleop_gated` prio 100, `drive` prio 10.

## Stan po naprawie (13.09, ta sama sesja)
Wszystkie punkty #1-#4 oraz #5/#7 naprawione na gałęzi `fix/sim-parity-20260913` (kod `src/sac_driver`,
`driver_params.yaml`, wagi `weights/*_policy.pth`, docs). Test offline `src/sac_driver/test/test_sim_parity.py`:
15/15 na nowym kodzie, 12/15 czerwone na starym. NIE jechało jeszcze na aucie - przed jazdą test kartonem
lewo/prawo (`docs/TROUBLESHOOTING.md`). Sekcje niżej opisują stan SPRZED naprawy.

## Werdykt

Sieć na aucie dostaje wektor stanu, który w trzech miejscach znaczy co innego niż w treningu, a do tego
skan lidaru obrócony o 90°. Model w symulacji jest dobry, ale na aucie widzi inny świat. Do tego wgrane wagi
to wczesne migawki słabszej sesji. Kolejność napraw: #1 → #2 → #3 → #4, potem test kartonem lewo/prawo.

## Błędy krytyczne (każdy z osobna wystarczy, żeby jazda była bez sensu)

### #1 Lidar obrócony o 90° - `lidar.angle_offset_deg: 0.0` jest zły dla tych modeli
- Symulator: **90° = przód** (`racer_env.py:15` `LIDAR_CENTER_DEG = 90.0`; docstring `build_lidar_angles`
  `racer_env.py:45` „Front hemisphere (0°-180°, centered on 90°=forward)"; `racer_env.py:261`
  `lidar_offsets = radians(90 - angle)`; `racer_env.py:1531,1562` `ray_angle = vehicle.angle + offset`).
  Tak było w KAŻDYM commicie (sprawdzone `git show` dla `749361e`, `19b4d58`, `d708499`).
- Auto: `lidar_converter.py:133` liczy kąt w ramce skanu jako `direction * (target + offset)` =
  `1.0 * (a + 0)` = **a**. W ROS 0 rad = przód, +90° = lewo. Czyli promień „przód" z sima (a=90°, indeks 180)
  czyta LEWY bok auta, a promień a=0° (w simie prawy bok) czyta PRZÓD.
- Konfig sprzed 26.03 (`driver_params_27ray.yaml`: offset -90, direction 1, steer_sign -1) był spójny.
  Commit `c15fe9a` zmienił offset na 0 z uzasadnieniem „0° = forward for the current models"
  (`.context/STATE.md:135,173`) - to zdanie jest sprzeczne z kodem sima.
- Dlaczego „test kartonem" (`KNOWLEDGE.md:85`: „offset=0 steered AWAY correctly") nie wykrył błędu: przy obrocie
  o 90° karton z przodu ląduje w simowym „prawym boku" (a≈0), sieć odbija w lewo (sim) = ujemny steer =
  auto skręca w prawo. Wygląda jak „ucieka od przeszkody". Test frontalny NIE rozróżnia obrotu o 90°.
- Poprawna zależność (wyprowadzenie): w simie dodatni steer zwiększa `angle` (`vehicle.py:283`), a promień a=0°
  leży po stronie, w którą auto wtedy skręca. W ROS dodatni `steering_angle` = lewo = +90°. Stąd sim a → ROS
  `90° - a`, czyli **`angle_direction: -1.0`, `angle_offset_deg: -90.0`, `steer_sign: 1.0`** (znak yaw z odom
  zostaje zgodny). Wariant alternatywny (offset -90, direction +1, steer_sign -1) odwraca znak kanału yaw.
- Zastrzeżenie: jeśli lidar fizycznie stoi obrócony o 90° względem `base_link` (static tf mówi yaw 0,
  `bringup_launch3.py:222-226`), offset 0 działa przypadkiem. Rozstrzyga test niżej („Test kartonem lewo/prawo").

### #2 Układ 5 skalarów w stanie nie zgadza się z treningiem (3 z 5 kanałów to inne wielkości)
| pozycja po 450 promieniach | trening (`racer_env.py:1765-1769`, niezmienione od lutego) | auto (`state_builder.py:70-72`, `sac_driver_node.py:436-441`) |
|---|---|---|
| +0 | `collision` 0/1 (w normalnej jeździe zawsze 0) | `speed_norm` 0..1 |
| +1 | `speed_norm` = \|v\| / max_speed, 0..1 | `steer_norm` -1..1 (ujemne wartości nigdy nie widziane w treningu) |
| +2 | `servo_norm` = (steer_cmd + 1)/2, **0..1, 0,5 = prosto** (`racer_env.py:1139,1724`) | `accel_feedback` -1..1 (wielkość, której w simie NIE MA - `git log -S accel_feedback` = 0 trafień) |
| +3 | `linear_accel / 4.0` | to samo ✅ |
| +4 | `angular_velocity / 3.0` | to samo ✅ |
- Skutek: sieć czyta skręt tam, gdzie uczyła się prędkości, a „ile mam skręcone" bierze z kanału, do którego auto
  wpisuje przyspieszenie (a to prawie stale ≈ +1, bo sieć nie hamuje - patrz #5). Notatka `.context/STATE.md`
  („Accel feedback channel interpretation … assumed") podejrzewała tylko jeden kanał.
- Źródło pomyłki: wiadomość `~/shared/inbox/pc-jetson/2026-03-26_inference_car_1_3.md` („State channels:
  speed_norm, steer_norm, accel_norm, linear_accel, angular_vel") opisuje format, którego sim nigdy nie miał.
  Ta sama notatka podaje `max_speed: 6.0` - stąd też błąd #3. Oba błędy weszły 1:1 do `driver_params.yaml`.
- Do tego znak i zakres feedbacku skrętu: auto liczy `(servo_raw - 0,535)/0,435` z topiku
  `/commands/servo/position`, a serwo = `-0,9·kąt + 0,5304` (`vesc.yaml:10-11`). Dla pełnego skrętu +20°
  wychodzi **-0,72**, a sim oczekuje **1,0** (w kanale +2). Poprawna wielkość to własna ostatnia akcja steer
  (po limicie tempa) przeliczona `(steer + 1)/2` - dokładnie tak liczy sim (`racer_env.py:1139`).

### #3 Dzielnik prędkości 6,0 zamiast ~2,5 - sieć „myśli", że jedzie 2,4× wolniej
- Oba modele z auta (`car_1_3`, `Rybnik_02_1`) trenowano 22-29.03, gdy `config/physics.yaml` miał
  `max_speed: 2.5` (`git show 19b4d58:config/physics.yaml`; 6,0 weszło dopiero 02.05 w `d708499`;
  06-21.03 było 4,0). DR skalowało 0,85-1,15 → 2,1-2,9 m/s.
- Auto: `state.max_speed_mps: 6.0` (`driver_params.yaml:21`). Przy 2 m/s sieć widzi 0,33 zamiast ~0,8.
- Z tego samego powodu `control.speed_limit_mps: 2.0` jest akurat blisko simowego maksimum - to zostawić.

### #4 Na aucie są wczesne migawki, i to najsłabszej sesji
| plik | total_steps na aucie | total_steps w finalnym pliku sesji | uwagi |
|---|---|---|---|
| `weights/session_Rybnik_02_1.pth` (AKTYWNY) | 122 293 | 794 552 (`runs/session_Rybnik_02_1/session_Rybnik_02_1.pth`) | md5 inne; sesja najsłabsza w rodzinie 450-ray: best mean_100 = 64,6 m, final = 22,7 m |
| `weights/session_car_1_3.pth` | 301 897 (epizod 5250/8750) | 866 051 | treść = `shared/różne/session_car_1_3.pth` = md5 `runs/session_car_1_3/session_car_1_3_Backup_5250.pth` (jedyne trafienie wśród 4947 plików .pth w `runs/`) |
- Ranking sesji 450-ray (front 0,5°/rear 2°, sieć [512,512,256], stack 4, action_repeat 8) po najlepszej
  `mean_100` (średni dystans ostatnich 100 epizodów, z CSV sesji; mapy różne, więc porównanie orientacyjne):
  `Mapa_4_1` 295 m (mapa K_01, physics max_speed 4,0) · `Mapa_3_3` 286 m (R_01, 4,0) · `Mapa_3_6` 252 m (R_01) ·
  `car_1_2` 220 m (R_01, 2,5) · `car_1_1` 206 m · `car_1_3` 183 m (R_01, 2,5) · **`Rybnik_02_1` 65 m**.
  Wniosek: aktywny model to nie „najlepszy trening", i wgrany jest z 15% jego treningu. Kandydat pod ten sam
  fizyczny reżim (max_speed 2,5 = limit auta 2 m/s): `car_1_2` albo finalny `car_1_3`.
- Commit `6f08f38` (08.09) przełączył domyślny model z `car_1_3` na `Rybnik_02_1` bez porównania w simie.

## Błędy średnie (nie tłumaczą „dużych dziwnych ruchów", ale psują jakość)

### #5 Czas i stack ramek inne niż w treningu
- Sim: decyzja co 8 klatek przy 60 fps = **7,5 Hz** (`config_sac_20.yaml:103`, `train_ssac.py:328-340`),
  stack = 4 KOLEJNE klatki po 16,7 ms (`racer_env.py:1956` dokłada obs co klatkę) = okno 67 ms, lidar
  przeliczany co klatkę (z opóźnieniem 3-4 klatek, `sensor_delay.lidar_delay_frames`).
- Auto: pętla **30 Hz** (`driver_params.yaml:31`), stack = 4 × 33 ms = 133 ms, a lidar S1 daje ~8-10 Hz -
  3-4 ramki w stacku niosą TEN SAM skan. Różnice między ramkami (czyli to, z czego sieć czyta ruch) są inne.
  Sensowny odpowiednik: decyzja i stack co ~8 Hz (np. nowa ramka tylko przy nowym skanie).

### #6 Sieć nie umie hamować, a auto integruje przyspieszenie w komendę prędkości
- Checkpointy: `action_scale=[1,1]`, `action_bias=[0,1]` → accel ∈ **[0, 2] m/s²** (bez ujemnych).
- Auto: `control_mapper.py:291` `speed_cmd = last + accel·dt` → monotonicznie do limitu 2 m/s i tam zostaje.
  W simie jest podobnie (tarcie działa tylko przy `accel_actual == 0.0`, `vehicle.py:250`), więc to cecha
  modelu, nie błąd integracji - ale znaczy, że auto na zakrętach nie zwolni. Do wiedzy przy strojeniu.
- Sprawdzone i OK: `control_mapper.reset(current_speed)` mnoży odom-ową prędkość przez `speed_sign=-1`;
  odom daje dodatnią prędkość do przodu (upstream `vesc_to_odom.cpp:102` neguje erpm), a komenda „do przodu"
  na tym aucie jest ujemna - znaki się zgadzają, integrator startuje poprawnie.

### #7 Brak krzywej skrętu zależnej od prędkości i inny rozstaw osi
- Sim: `steer_limit = 20° + (5° - 20°)·(v/8)²` (`vehicle.py:220-226`). Przy max 2,5 m/s efekt mały (≥18,5°),
  więc dla modeli marcowych to drobiazg; dla modeli po 02.05 (max 6 m/s) będzie 2-3× za duży skręt.
- Wheelbase: sim 0,27 m (`params.py:15` = length·0,6, DR 0,23-0,31), auto 0,35 m (`driver_params.yaml:42`,
  `vesc.yaml:39`) - poza zakresem randomizacji.
- Limity tempa: sim steer 0,1/klatkę = 6 jedn./s (≈120°/s), accel 0,3/klatkę = 18 m/s³; auto 90°/s i 3 m/s³ -
  auto reaguje wolniej niż to, czego sieć się nauczyła.

## Drobne / do wiedzy
- `safe_mode: true` nic nie robi (`safe_steer_scale`/`safe_accel_scale` = 1,0).
- `_data_ready` wymaga świeżego `/commands/servo/position` (≤0,5 s), a ten topic publikuje się tylko, gdy ktoś
  jedzie - start działa dzięki jednorazowemu „stop", ale to krucha zależność (opisane w TROUBLESHOOTING).
- `linear_accel` liczony z różnicy prędkości odom po czasie przyjścia wiadomości, nie po stemplu - szum.
- Dokumentacja auta (README/DOCUMENTATION/STATE/KNOWLEDGE) opisuje format stanu z #2 jako fakt - po naprawie
  poprawić, inaczej następna sesja odtworzy błąd.
- Sesje „Mapa_64_*" (luty, best do 577 m) mają inny wymiar wejścia (stary lidar) - nie pasują do 1820-dim.

## Test kartonem lewo/prawo (rozstrzyga #1 bez jazdy, koła w górze)
1. `ros2 topic echo /scan` + `python3 ros2_panel/scan_test.py`: karton 0,5 m PRZED lidarem → najbliższy punkt
   ma mieć kąt ≈ 0°; karton po LEWEJ stronie auta → ≈ +90°. Jeśli przód wychodzi na ±90° lub 180°, lidar stoi
   obrócony mechanicznie i trzeba to odjąć w offsetcie ORAZ w static tf.
2. Po ustawieniu `direction -1 / offset -90`: w `sac_driver` zalogować indeks minimum wektora 450 - karton z przodu
   → indeks 180 (a=90°); karton po lewej → indeks 0 (a=0°); po prawej → indeks 360 (a=180°).
3. Karton po LEWEJ z przodu → auto (RB, koła w górze) ma skręcać w PRAWO (`/commands/servo/position` > 0,53);
   po PRAWEJ → w LEWO. Test frontalny sam nic nie dowodzi (patrz #1).

## Czego nie zmierzyłem
- Fizycznego montażu lidaru i tego, co pokazał marcowy test kartonem (tylko notatki).
- Zachowania na żywo po naprawie - wymaga auta.
- Pochodzenia `best_mapa1_ep18000_clean.pth` (stary model 128-dim, nieaktywny profil `driver_params_27ray.yaml`).
