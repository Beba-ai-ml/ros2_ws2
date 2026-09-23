# HANDOFF 13.09.2026 - naprawa zgodności sim↔auto (gałąź `fix/sim-parity-20260913`)

**Uwaga 2026-09-23:** opisy montażu 180°, offsetu +90 i zaliczonych testów poniżej są
historyczne i nie opisują obecnego stanu. Live/YAML ma -90/-1, TF nadal π, a cztery testy
lidarowe nie przechodzą. Najpierw przeczytaj [bieżący research](RESEARCH-jetson-20260923.md).

Dla Wojtka i dla następnej sesji agenta. Ten handoff opisuje naprawę z 13.09.2026; bieżący
domyślny model został później przełączony na `session_Sesja_mpo2_2_policy.pth` (stan bieżący
jest w `.context/STATE.md`). Co zrobiono, co czeka, co zdecydowano. Dowody na każdy punkt:
`.context/review-jazda-ai-20260913.md` (plik:linia w simie i na aucie).

## Skąd się wzięło
Wojtek: „w symulacji działa perfekcyjnie, na aucie robi duże dziwne rzeczy". Review 13.09 znalazł, że
węzeł `sac_driver` karmił sieć innym światem niż trening (`~/occupancy_racer/Soft_Actor_Critic_2`,
HEAD `5b59934` = GitHub `occupancy-racer-sac2`). Cztery twarde rozjazdy, każdy sam wystarczał:
1. skan lidaru obrócony o 90° (sim: 90° = przód; auto miało offset 0);
2. kanały stanu po lidarze `[speed, steer -1..1, accel_feedback]` zamiast simowych `[kolizja, speed, serwo 0..1]`
   (format wzięty z błędnej notatki z inboxa 26.03, sim nigdy takiego nie miał);
3. dzielnik prędkości 6,0 zamiast 2,5 (physics.yaml w czasie treningu marcowych modeli);
4. na aucie wczesne migawki (Rybnik 122k z 795k kroków; najsłabsza sesja rodziny, mean_100 65 m).

## Co jest zrobione (commit `b1c8673` na gałęzi, wypchnięty na GitHub, NIE scalony do `main`)
- `driver_params.yaml`: for the later-confirmed physical 180° lidar mount, `lidar.angle_offset_deg +90`,
  `lidar.angle_direction -1`, `state.max_speed_mps 2.5`,
  `control.rate_hz 60` + `control.decision_every_n 8`, krzywa skrętu (`min_steering_angle_deg 5`,
  `steer_speed_ref_mps 8`), limity tempa jak w simie (120°/s, 18 m/s³). `speed_sign -1` i `steer_sign +1` bez zmian.
- `state_builder.py`: układ `[lidar, kolizja=0, |v|/2.5, (steer+1)/2, accel/4, yaw/3]`, stack 4 ramek najstarsza pierwsza.
- `sac_driver_node.py`: tick 60 Hz, obserwacja co tick, sieć co 8 ticków, akcja trzymana pomiędzy; feedback skrętu
  z własnej ostatniej komendy (`ControlMapper.last_steer_norm`); subskrypcja `/commands/servo/position` usunięta,
  `_data_ready` czeka tylko na `/scan` i `/odom`.
- `control_mapper.py`: limit skrętu zależny od prędkości jak `vehicle.py` w simie; `last_steer_norm`.
- Wagi w czasie tego historycznego handoffu: `weights/session_car_1_2_policy.pth` (R_01,
  best mean_100 220 m) i `weights/session_car_1_3_final_policy.pth` (R_01 + bot, 8750 epizodów).
  Bieżący default jest opisany wyżej i w `.context/STATE.md`. Sama polityka, ~5 MB, format legacy
  torch (czytelny dla torch 1.13 na Jetsonie). Źródło: `Soft_Actor_Critic_2/runs/<sesja>/<sesja>.pth` (finalne).
  Stare `session_Rybnik_02_1.pth` i `session_car_1_3.pth` (migawka epizodu 5250) usunięte z repo (są w historii gita).
- Test offline `src/sac_driver/test/test_sim_parity.py` (bez ROS): 15/15 na nowym kodzie, 12/15 czerwone na
  starym `main` (kontrola negatywna). Uruchomienie: `cd src/sac_driver && python3 test/test_sim_parity.py`.
- Docs poprawione: README, DOCUMENTATION, `.context/KNOWLEDGE.md`, `.context/STATE.md`, `docs/TROUBLESHOOTING.md`,
  `install.sh` (nazwa wag), `tools/lidar_diag.py`, `driver_params_27ray.yaml` oznaczony jako archiwalny (niezgodny z węzłem).

## Co czeka na Wojtka (w tej kolejności)
- [ ] **H1. Wgrać gałąź na Jetsona:** `cd ~/ros2_ws && git fetch && git checkout fix/sim-parity-20260913 &&
  colcon build --packages-select sac_driver && . install/setup.bash`. Panel „AI Inference" buduje i odpala sam.
- [ ] **H2. Koła w górę, test kartonem LEWO/PRAWO** (`docs/TROUBLESHOOTING.md`, rozdział „Car steers into obstacles"):
  1. `python3 ros2_panel/scan_test.py`: przy fizycznym montażu 180° karton z przodu → najbliższy punkt ~±180°;
     z lewej → ~-90°. Aktywny launch ma teraz static TF `base_link -> laser` yaw π, a AI offset +90°.
  2. Z węzłem AI: karton z przodu = minimum wektora 450 na indeksie 180, z lewej na 0, z prawej na 360.
  3. RB wciśnięty: karton z przodu-lewej → koła w PRAWO (`/commands/servo/position` > 0,53), z przodu-prawej → w LEWO.
     Lustrzanie przy zaliczonych 1-2 = zły znak `steering_angle_to_servo_gain` w `vesc.yaml` (dodatni kąt musi
     skręcać w lewo, inaczej yaw z `/odom` też jest lustrzany). NIE naprawiać tego samym `steer_sign`.
  Test tylko z przodu nic nie dowodzi - tak marcowy „test kartonem" przepuścił obrót o 90°.
- [ ] **H3. Pierwsza jazda po ziemi** przy `control.speed_limit_mps 2.0` na bieżącym
  `session_Sesja_mpo2_2_policy.pth`; R_01 można porównać przez `session_car_1_2_policy.pth` (przełączenie:
  `model.path` w `driver_params.yaml` albo
  `ros2 launch sac_driver sac_driver.launch.py model_path:=...`).
- [ ] **H4. Po udanej jeździe: scalić gałąź do `main`** (PR: https://github.com/Beba-ai-ml/ros2_ws2/pull/new/fix/sim-parity-20260913)
  i dopisać wynik do `.context/STATE.md` (czerwony blok na górze zdjąć).
- [ ] **H5. Jeśli zakręty są słabe mimo zgodności:** rozstaw osi auta 0,35 m vs 0,27 m w treningu (DR 0,23-0,31) -
  dotrenować z `wheelbase: 0.35` w `physics.yaml` sima. Sieć nie umie hamować (accel 0..2) - to cecha treningu,
  nie błąd integracji.

## Decyzje podjęte 13.09 (nie otwierać ponownie)
- Naprawa poszła w tej samej sesji co review (Wojtek: „czy dasz radę to naprawić" → tak).
- Domyślny model = `car_1_2` (najlepszy wynik w simie w tej samej fizyce co limit auta), `car_1_3` finalny jako drugi.
- Nie zmieniono kalibracji `vesc.yaml`, `speed_sign`, `steer_sign` (reguła AGENTS.md pkt 4); `lidar.angle_offset_deg`
  i `angle_direction` zmienione na podstawie kodu sima - fizyczny re-test to H2.
- Stare notatki (`KNOWLEDGE.md` sprzed 13.09, inbox 26.03) o „0° = przód" i „5 kanałów bez kolizji" są błędne;
  źródłem prawdy jest `racer_env.py` w simie, nie dokumentacja auta.

## Czego nie zweryfikowano
- Żywego auta (brak dostępu z PC). Cały dowód to analiza kodu + test offline + symulacja 16 ticków bez ROS.
- Żywego odczytu kartonu z `/scan` po zmianie (montaż 180° został potwierdzony przez użytkownika;
  poprzedni static TF mówił yaw 0 i został skorygowany lokalnie na π).
- Pochodzenia `best_mapa1_ep18000_clean.pth` (stary model 128-dim, nieaktywny).
