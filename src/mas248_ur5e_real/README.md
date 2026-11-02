# mas248_ur5e_real (L10): den ekte UR5e-en

Pakken kommer fra L10-zipen på Canvas. Zipen hadde tomme `scripts/` og `tests/`, så gruppa (med AI, slik foreleser ba om) har skrevet de fem skriptene som `CMakeLists.txt` krever:

| Skript | Hva det gjør | Beveger roboten? |
|---|---|---|
| `prepare_description.py` | Kjøres av CMake. Lager `ur5e_workcell_real.urdf.xacro` i `build/` fra L8-workcellen vår (samme pedestal, spindel og TCP), men med UR-driverens ekte `ros2_control` og kalibrering. Kopierer også vendor-YAML fra `ur_description`. | Nei |
| `validate_description.py` | Offline-sjekk: rendrer modellen med kalibreringsfila, akkurat slik launch-filene gjør. | Nei |
| `validate_stack.py` | Live-sjekk (L10, slide 16 og 21): `/joint_states`, kontrollere, MoveIt, `robot_program_running`, robot- og sikkerhetsmodus, speed scaling. | Nei |
| `preflight.py` | Kjører `validate_stack` og deretter `run_picture_real` i plan-only. | Nei |
| `run_picture_real.py` | Tegner `targets`-JSON med samme planlegging som L9 `run_picture`. Standard er PLAN ONLY. | Bare med `--execute --acknowledge-hardware --acknowledge-clearance`, og med Enter før hver bane |

I tillegg er det gjort to små endringer i kursfilene:

- **`launch/_real_common.py`:** sjekken krevde linken `spindle_body`. Verktøylinken vår heter `spindle`, og SRDF-en bruker det navnet, så nå godtas begge.
- **`CMakeLists.txt`:** generatoren kjøres på nytt når `ur5e_workcell.urdf.xacro` endres. Uten dette ble en gammel avledet modell liggende igjen i `build/`.

Alle kjøringer av `run_picture_real.py` lagrer bevis i `~/mas248_runs/<tidspunkt>/`: `targets.json`, `plan.json`, `summary.json` og `joint_states.csv` (målte ledd under kjøring). Dette er rådata til resultatkapittelet i rapporten (planlagt mot utført bane).

Gruppa bruker **ROS_DOMAIN_ID=57** overalt. Verdien 48 i L10-slidene er bare en plassholder.

---

## 0. Én gang: avhengigheter, kalibrering og bygg (hjemme)

```bash
sudo apt install ros-humble-ur          # ur_robot_driver, ur_description, ur_calibration, ur_client_library
cp ~/Nedlastinger/ur5e_calibration.yaml ~/ur5e_calibration.yaml
chmod +x ~/mas248_ws/src/mas248_ur5e_real/scripts/*.py   # nødvendig med --symlink-install

source /opt/ros/humble/setup.bash
cd ~/mas248_ws
colcon build --symlink-install
source install/setup.bash

ros2 run mas248_ur5e_real validate_description.py --calibration ~/ur5e_calibration.yaml
```

Forventet resultat: tre `[OK]`-linjer og `Description check passed.`

## 1. Før labøkta: mål papiret og bordet

Bordet i L8-workcellen (x = 0.25–1.45 m, toppflate z = 0, altså gulvnivået pedestalen står på, og virtuelle vegger ved y = ±0.60 m) er et kollisjonsobjekt for MoveIt. Hvis det ikke stemmer med det ekte bordet, får dere enten falske kollisjoner eller ingen beskyttelse.

På lab:

1. Kjør pennetuppen forsiktig ned til papiret med freedrive på pendanten.
2. Les av posisjonen med `ros2 run tf2_ros tf2_echo world tcp` mens driveren går.
3. Mål minst tre hjørner. Da får dere papirets senter (x, y) og høyde (z).
4. Juster bordet i `mas248_ur5e/urdf/ur5e_workcell.urdf.xacro` slik at toppflata ligger 3–5 mm *under* papirets z, og bygg på nytt. Hvis bordet ligger nøyaktig på papiret, kan MoveIt telle pennens berøring som kollisjon.

> 📝 **Notat til rapporten:** skriv opp de målte koordinatene, hvordan dere målte dem og avviket fra L9-antakelsen.

## 2. Nettverk (lab-PC-en, én gang)

```bash
# Robot 192.168.1.101, PC 192.168.1.102/24
ping -c 2 192.168.1.101
sudo ufw allow 50001/tcp; sudo ufw allow 50002/tcp; sudo ufw allow 50003/tcp; sudo ufw allow 50004/tcp
```

## 3. Oppstart på lab, først uten bevegelse

**Terminal 1, driveren:**

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=57

ros2 launch mas248_ur5e_real driver.launch.py \
  robot_ip:=192.168.1.101 \
  reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=false \
  acknowledge_hardware:=yes
```

**Pendanten:** Last inn External Control-programmet (Host IP 192.168.1.102, port 50002), velg Local mode og trykk Play *etter* at driveren kjører.

**Terminal 2, MoveIt og RViz:**

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=57

ros2 launch mas248_ur5e_real moveit_real.launch.py \
  robot_ip:=192.168.1.101 \
  reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=false \
  launch_rviz:=true
```

Hvis RViz får "double -> string"-feilen igjen (norsk locale), start terminal 2 med `LC_NUMERIC=C ros2 launch ...`.

**Terminal 3, sjekk og mål:**

```bash
source /opt/ros/humble/setup.bash; source ~/mas248_ws/install/setup.bash; export ROS_DOMAIN_ID=57
ros2 run mas248_ur5e_real validate_stack.py
```

## 4. Mål og preflight, fortsatt uten bevegelse

```bash
# Bruk de målte verdiene fra punkt 1. --tcp-offset-mm 0 betyr at pennen akkurat berører papiret.
ros2 run mas248_ur5e_demos image_to_targets ~/mas248_ws/src/mas248_ur5e_demos/images/pumpkin_outline.png \
  --center-x <X> --center-y <Y> --center-z <Z_PAPIR> --width-m 0.12 \
  --output /tmp/mas248_l10_targets_lower.json

ros2 run mas248_ur5e_demos show_targets_rviz --json /tmp/mas248_l10_targets_lower.json --no-orientations
ros2 run mas248_ur5e_real preflight.py --targets /tmp/mas248_l10_targets_lower.json
```

Forventet slutt: `Planning OK`, `Collisions OK`, `Trajectory valid`, `Preflight check passed!` og `PREFLIGHT PASSED`.

## 5. Fysisk kjøring, bare med faglærer til stede

1. Stopp terminal 1 og 2 og start dem på nytt med `enable_execution:=true`. Trykk Play på pendanten igjen.
2. Sett hastighetsslideren på pendanten til 10–20 %.
3. Kjør `ros2 run mas248_ur5e_real validate_stack.py --execution`. Alle punktene skal være `[OK]`.
4. Start med ett strøk:

```bash
ros2 run mas248_ur5e_real run_picture_real.py --targets /tmp/mas248_l10_targets_lower.json \
  --stroke 1 --execute --acknowledge-hardware --acknowledge-clearance
```

Skriptet spør om Enter før hver bane, og `q` stopper. Ctrl+C avbryter banen som kjører. Nødstoppen på pendanten er alltid den egentlige sikkerheten.

Når første strøk ser riktig ut, kjører dere hele bildet (uten `--stroke`) og deretter deres eget bilde.

> 📝 **Notat til rapporten:** for hver kjøring, noter mappa i `~/mas248_runs/`, hastighet på slideren, om pennen traff papiret, og ta bilde eller video av tegningen. Kjør samme bilde flere ganger for repeterbarhet (L10, slide 42).

## Hva er ikke testet

Skriptene er testet offline: xacro-generering og -rendering mot `ur_description` (Humble), og sjekklogikken for baner. Det som gjenstår er kjøring mot ROS og en ekte robot. Første gang dere bygger, se etter `L10 workcell generated: ...` i colcon-loggen. Står det `not supported by installed macro, ignored: ...`, er `ros-humble-ur` gammel, og da bør dere kjøre `sudo apt upgrade`.
