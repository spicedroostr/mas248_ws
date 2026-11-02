# mas248_ur5e_real (L10): den ekte UR5e-en

Pakken kommer fra L10-zipen på Canvas. Zipen hadde tomme `scripts/` og `tests/`, så gruppa (med AI, slik foreleser ba om) har skrevet de fem skriptene som `CMakeLists.txt` krever:

| Skript | Hva det gjør | Beveger roboten? |
|---|---|---|
| `prepare_description.py` | Kjøres av CMake. Lager `ur5e_workcell_real.urdf.xacro` i `build/` fra L8-workcellen vår (samme pedestal, spindel, TCP, bord, vegger og A4-ark), men med UR-driverens ekte `ros2_control` og kalibrering. Kopierer også vendor-YAML fra `ur_description`. | Nei |
| `validate_description.py` | Offline-sjekk: rendrer modellen med kalibreringsfila, akkurat slik launch-filene gjør. | Nei |
| `validate_stack.py` | Live-sjekk (L10, slide 16 og 21): `/joint_states`, kontrollere, MoveIt, `robot_program_running`, robot- og sikkerhetsmodus, speed scaling. | Nei |
| `preflight.py` | Kjører `validate_stack` og deretter `run_picture_real` i plan-only. | Nei |
| `run_picture_real.py` | Tegner `targets`-JSON med samme planlegging som `run_picture` i mock (Kartesisk forflytning mellom strøk, løft og nye forsøk på hjem-banen). Standard er PLAN ONLY. | Bare med `--execute --acknowledge-hardware --acknowledge-clearance`, og med Enter før hver bane |

Endringer i kursfilene:

- **`launch/_real_common.py`:** sjekken krevde linken `spindle_body`. Verktøylinken vår heter `spindle`, så nå godtas begge.
- **`launch/moveit_real.launch.py`:** RViz startes med `LC_NUMERIC=C` (norsk desimalkomma knekker MotionPlanning-pluginen, samme fiks som i mock).
- **`CMakeLists.txt`:** generatoren kjøres på nytt når `ur5e_workcell.urdf.xacro` endres. Uten dette ble en gammel avledet modell liggende igjen i `build/`.

Alle kjøringer av `run_picture_real.py` lagrer bevis i `~/mas248_runs/<tidspunkt>/`: `targets.json`, `plan.json`, `summary.json` og `joint_states.csv` (målte ledd under kjøring). Dette er rådata til resultatkapittelet (planlagt mot utført bane).

Gruppa bruker **ROS_DOMAIN_ID=57** overalt. Verdien 48 i L10-slidene er bare en plassholder.

**Oversikt over økta:** 0 hjemme → 1 nettverk → 2 pendant → 3 rydd opp → 4 start uten bevegelse → 5 mål arket → 6 mål og preflight → 7 slå på bevegelse → 8 første bevegelse i lufta → 9 tegn → 10 avslutt. Ved feil, se **Feilsøking** nederst.

---

## 0. Hjemme, én gang: avhengigheter, kalibrering og bygg

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

Forventet resultat: `[OK]`-linjer og `Description check passed.` I colcon-loggen skal det stå `L10 workcell generated: ...`.

Test at mock fortsatt virker (se `kjoreguide_mock.md`) og at dere har en targets-fil for pumpkin og Ubuntu-logoen. Ta med lader, nettverkskabel og en tusj som sitter fast i verktøyet.

## 1. Nettverk på lab, én gang

Koble kabelen direkte fra PC-en til roboten. Finn navnet på kablet nettverkskort (for eksempel `enp3s0`):

```bash
ip -br link
```

Gi PC-en fast adresse 192.168.1.102 på det kortet (bytt `enp3s0` med deres navn):

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name ur5e ipv4.method manual ipv4.addresses 192.168.1.102/24
sudo nmcli con up ur5e
ping -c 3 192.168.1.101
```

Åpne portene driveren bruker (bare nødvendig hvis `ufw` er aktiv, sjekk med `sudo ufw status`):

```bash
sudo ufw allow 50001:50004/tcp
```

`ping` skal svare. Hvis ikke, se Feilsøking A.

## 2. Pendanten

1. Sjekk robotens IP under *Settings → System → Network*: statisk, 192.168.1.101, maske 255.255.255.0.
2. Sjekk at URCap-en **External Control** er installert (*Settings → System → URCaps*). Under *Installation → URCaps → External Control*: Host IP **192.168.1.102**, port **50002**.
3. Lag (eller last inn) et program som bare inneholder noden *External Control*. Lagre det, for eksempel som `mas248_ros.urp`.
4. Under *Installation → General → TCP/Payload*: sett payload til omtrent vekten av verktøyet (spindel og tusj). Feil payload gir unødvendige protective stops.
5. Sett hastighetsslideren nederst på skjermen til **10–20 %**.
6. Sjekk at roboten er slått på, bremsene er sluppet (*Normal*) og at nødstoppen er tilgjengelig for den som står ved roboten.

> 📝 **Notat til rapporten:** robotens IP-oppsett, URCap-versjon, payload og hastighet på slideren.

## 3. Rydd opp før dere starter (hver gang)

Mock og ekte robot kan ikke kjøre samtidig. De bruker samme nodenavn og samme `ROS_DOMAIN_ID`.

```bash
pkill -f ros2_control_node; pkill -f move_group; pkill -f rviz2; pkill -f show_targets_rviz
pkill -f run_picture; pkill -f ur_ros2_control_node; pkill -f robot_state_publisher
ros2 daemon stop
ros2 node list        # skal være tom
```

Hver terminal eller tmux-fane trenger dette (eller legg det i `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=57
```

## 4. Start uten bevegelse

**Fane 1, driveren:**

```bash
ros2 launch mas248_ur5e_real driver.launch.py \
  robot_ip:=192.168.1.101 \
  reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=false \
  acknowledge_hardware:=yes
```

Vent til loggen er rolig og kontrollerne er lastet. **Pendanten:** last inn `mas248_ros.urp` og trykk **Play** først nå, etter at driveren kjører. I fane 1 skal det komme `Robot connected to reverse interface. Ready to receive control commands.`

**Fane 2, MoveIt og RViz:**

```bash
ros2 launch mas248_ur5e_real moveit_real.launch.py \
  robot_ip:=192.168.1.101 \
  reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=false \
  launch_rviz:=true
```

Vent på `You can start planning now!`. Argumentene for fane 1 og 2 må være like (samme IP, kalibrering og `enable_execution`).

**Fane 3, sjekk:**

```bash
ros2 run mas248_ur5e_real validate_stack.py
```

Alle påkrevde punkter skal være `[OK]`, og det skal stå `Stack check passed.`

**Se i RViz:** roboten skal stå i samme stilling som den ekte. Flytt et ledd litt med pilene på pendanten (programmet på pause) og se at RViz følger etter. Spindelen og pennen skal peke samme vei som på den ekte roboten. Hvis ikke, se Feilsøking D.

## 5. Mål arket (fortsatt uten bevegelse fra ROS)

1. Sett programmet på pendanten på **pause/stopp**. Driveren og `/joint_states` fortsetter å virke.
2. Bruk **Freedrive** (knappen bak på pendanten) og før pennetuppen forsiktig ned til hvert hjørne av arket.
3. Les av posisjonen i fane 3 for hvert hjørne:

   ```bash
   ros2 run tf2_ros tf2_echo world tcp
   ```

   Noter `Translation: [x, y, z]`. Ta minst tre hjørner, gjerne alle fire, pluss midten.
4. Regn ut:
   - **Senter x, y:** gjennomsnittet av hjørnene.
   - **Yaw:** vinkelen til arkets langside mot world-Y. Er den under 2°, la den være 0.
   - **z_papir:** gjennomsnittet av z-verdiene. Den tar også med seg eventuelle feil i TCP-lengden i modellen, så **bruk den målte z-en direkte** og ikke 0.
5. Oppdater `mas248_ur5e/urdf/ur5e_workcell.urdf.xacro`:
   - `paper_center_x`, `paper_center_y`, `paper_yaw` (radianer) med de målte verdiene. Arket er bare visuelt.
   - `table_top_z`: kollisjonsmodellen til pennen slutter 10 mm over tuppen, så bordet kan ligge i z = 0 så lenge `z_papir` er mellom −0.005 og +0.005. Ligger z_papir utenfor, settes `table_top_z = z_papir − 0.003`.
6. Bygg og start fane 1 og 2 på nytt (rekkefølge som i punkt 3 og 4):

   ```bash
   cd ~/mas248_ws && colcon build --symlink-install && source install/setup.bash
   ```

> 📝 **Notat til rapporten:** de målte hjørnene, utregnet senter, yaw og z_papir, og avviket fra antakelsen i mock (x = 0.4885, z = 0.001).

## 6. Lag mål og kjør preflight (planlegger, beveger ingenting)

Fane 3. Begynn med en **luft-versjon** 20 mm over arket (`--tcp-offset-mm 20`) og en versjon på arket. Bytt inn de målte tallene:

```bash
X=0.4885; Y=0.0; Z=0.001; YAW=0     # målt i punkt 5 (YAW i grader)

ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.png --width-m 0.12 \
  --center-x $X --center-y $Y --center-z $Z --yaw-deg $YAW --tcp-offset-mm 20 \
  --output /tmp/mas248_l10_air.json
ros2 run mas248_ur5e_demos image_to_targets pumpkin_outline.png --width-m 0.12 \
  --center-x $X --center-y $Y --center-z $Z --yaw-deg $YAW \
  --output /tmp/mas248_l10_targets_lower.json

ros2 run mas248_ur5e_demos show_targets_rviz --json /tmp/mas248_l10_targets_lower.json   # fane 4, kan stå på
ros2 run mas248_ur5e_real preflight.py --targets /tmp/mas248_l10_air.json
ros2 run mas248_ur5e_real preflight.py --targets /tmp/mas248_l10_targets_lower.json
```

Forventet slutt på begge: `PREFLIGHT PASSED`. Banen skal ligge oppå arket i RViz. Gjenta med `ubuntu_logo_bw.png` (lagre som for eksempel `/tmp/mas248_l10_ubuntu.json`).

## 7. Slå på bevegelse (bare med faglærer til stede)

1. Ctrl+C i fane 2, deretter fane 1. Stopp programmet på pendanten.
2. Start fane 1 og fane 2 på nytt som i punkt 4, men med **`enable_execution:=true` i begge**.
3. Trykk **Play** på pendanten etter at driveren kjører.
4. Sjekk at slideren står på 10–20 %.
5. Kjør sjekken med kravene for bevegelse:

   ```bash
   ros2 run mas248_ur5e_real validate_stack.py --execution
   ```

   Alle punktene skal være `[OK]`.

## 8. Første bevegelse: ett strøk i lufta

Én person står ved nødstoppen, og ingen står innenfor rekkevidde.

```bash
ros2 run mas248_ur5e_real run_picture_real.py --targets /tmp/mas248_l10_air.json \
  --stroke 1 --execute --acknowledge-hardware --acknowledge-clearance
```

Skriptet viser hver bane (hjem, tilnærming, strøk, løft) og venter på **Enter**. Skriv `q` for å stoppe før neste bane. Ctrl+C avbryter banen som kjører. Nødstoppen på pendanten er alltid den egentlige sikkerheten.

Sjekk at roboten går først til home, så rett ned over strøket, tegner 20 mm over arket og løfter. Ingen snurring i håndleddet. Kjør deretter hele luft-bildet (uten `--stroke 1`).

## 9. Tegn på arket

```bash
# ett strøk først
ros2 run mas248_ur5e_real run_picture_real.py --targets /tmp/mas248_l10_targets_lower.json \
  --stroke 1 --execute --acknowledge-hardware --acknowledge-clearance
# hele bildet
ros2 run mas248_ur5e_real run_picture_real.py --targets /tmp/mas248_l10_targets_lower.json \
  --execute --acknowledge-hardware --acknowledge-clearance
```

- **Tusjen tegner ikke:** lag nye mål med `Z` 1 mm lavere og prøv ett strøk. Gå aldri mer enn 2 mm under målt z_papir.
- **Tusjen presser hardt eller roboten får protective stop:** øk `Z` med 1 mm.
- Vent til skriptet er ferdig og roboten står i home før neste kjøring.
- Når dere er trygge, kan `--no-confirm` brukes for en sammenhengende kjøring (fin til videoen).

Gjør pumpkin først (L10 minimumseksperiment), deretter Ubuntu-logoen. Kjør samme bilde 3 ganger på samme ark eller nye ark for repeterbarhet.

> 📝 **Notat til rapporten:** for hver kjøring, noter mappa i `~/mas248_runs/`, hastighet på slideren, Z-verdien, om pennen traff papiret, og ta bilde eller video av tegningen. Mål avvik på arket (for eksempel diameter på sirklene) mot bildet.

## 10. Avslutt

1. Vent til roboten står stille i home.
2. Ctrl+C i fane 4 og 3, deretter fane 2 og fane 1.
3. Stopp programmet på pendanten.
4. Kopier `~/mas248_runs/` til en sikker plass (for eksempel Overleaf-prosjektet eller en minnepinne).

---

## Feilsøking

### A. Nettverk

| Symptom | Årsak og løsning |
|---|---|
| `ping 192.168.1.101` svarer ikke | Feil nettverkskort eller adresse. Sjekk `ip -br addr` (skal vise 192.168.1.102/24 på kablet kort), kabel og at roboten har 192.168.1.101. Slå av Wi-Fi hvis det også bruker 192.168.1.x. |
| Fane 1 henger på `Connecting to ...` eller `Connection refused` | Roboten er ikke startet eller har feil IP. Pendanten må være forbi oppstartsbildet. |
| Play på pendanten gir *"Could not connect to host"* / *socket error* | Host IP i External Control er ikke 192.168.1.102, port er ikke 50002, eller `ufw` blokkerer. Kjør `sudo ufw allow 50001:50004/tcp`. Start driveren **før** Play. |
| `Connection to reverse interface dropped` | Programmet på pendanten er stoppet eller nettverket falt ut. Trykk Play igjen. |

### B. Driver og kontrollere

| Symptom | Årsak og løsning |
|---|---|
| `Explicit acknowledge_hardware:=yes required` | Glemt argumentet i fane 1. |
| Feil om kalibrering / filen finnes ikke | Bruk full sti: `calibration:="$HOME/ur5e_calibration.yaml"`. |
| `The calibration parameters of the connected robot don't match the ones from the given kinematics config file` | Kalibreringsfila er fra en annen robot. Lag en ny fra denne roboten: `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.101 target_filename:="$HOME/ur5e_calibration.yaml"`, bygg og start på nytt. |
| Spawner timeout / `Could not contact service /controller_manager/...` | Gamle prosesser fra mock. Gjør punkt 3 (rydd opp) og start på nytt. |
| `validate_stack`: `robot_program_running is not true` | Trykk Play på pendanten (etter at driveren kjører). |
| `validate_stack --execution`: `scaled_joint_trajectory_controller active (state: inactive)` | Fane 1 ble startet med `enable_execution:=false`. Start på nytt med `true` (punkt 7). |
| `competing controllers active` | En annen kontroller er aktiv. `ros2 control list_controllers`, deretter `ros2 control switch_controllers --deactivate <navn>`. |
| `/joint_states missing/stale` | Feil `ROS_DOMAIN_ID` i den terminalen (`echo $ROS_DOMAIN_ID` skal gi 57), eller driveren kjører ikke. |

### C. MoveIt og planlegging

| Symptom | Årsak og løsning |
|---|---|
| RViz uten MotionPlanning eller feilen *"double -> string"* | Skal være fikset (LC_NUMERIC=C i launch-fila). Bygg på nytt hvis den fortsatt dukker opp. |
| `Start state is in collision` / `Found a contact between 'tcp'/'spindle' and 'table_link'` | Bordet i modellen ligger for høyt i forhold til målt arkhøyde. Sett `table_top_z = z_papir − 0.003` (punkt 5) og bygg på nytt. Mot `workspace_walls`: arket eller roboten står for nær kanten av 120 cm-sonen. |
| Kartesisk bane `fraction < 0.999` eller `joint jump` | Målet er utenfor rekkevidde eller nær en singularitet. Flytt senteret nærmere roboten (mindre x) eller reduser `--width-m`. |
| `MoveItErrorCodes=99999` eller `-1` | OMPL er tilfeldig. Skriptet prøver hjem-banen opptil 5 ganger. Feiler det fortsatt, kjør kommandoen på nytt. |
| Tegningen havner på feil sted i RViz | `show_targets_rviz` viser gammel fil. Ctrl+C og start den på nytt etter ny `image_to_targets`. |

### D. Utførelse på den ekte roboten

| Symptom | Årsak og løsning |
|---|---|
| `Start state deviates` / `start tolerance` | Roboten er flyttet (freedrive) etter planleggingen. Kjør skriptet på nytt, det planlegger fra den nye stillingen. |
| MoveIt `CONTROL_FAILED` eller bane avbrytes med en gang | Programmet på pendanten er stoppet, eller slideren står på 0 %. Trykk Play og sjekk slideren. |
| Roboten går veldig sakte eller stopper litt | Normalt med lav slider: `scaled_joint_trajectory_controller` skalerer tiden. Skru opp slideren gradvis (maks ca. 50 % på lab). |
| Protective stop under tegning | Tusjen presser for hardt (øk `Z` med 1 mm), feil payload (punkt 2) eller kollisjon. Bekreft på pendanten, før armen opp med freedrive, trykk Play og kjør på nytt. |
| RViz-roboten står annerledes enn den ekte, eller pennen peker feil vei | Feil kalibrering eller feil verktøymodell. Stopp, ikke kjør. Sjekk `validate_description` og verktøyets montering mot SolidWorks-modellen. |
| Tegningen er forskjøvet eller rotert på arket | Senter eller yaw er feil målt. Mål på nytt (punkt 5). Rotasjon rettes med `--yaw-deg`. |
| Tegningen er tydelig i én del og svak i en annen | Arket eller bordet heller. Mål z i alle fire hjørner. Legg arket flatt, eller bruk gjennomsnittet og la tusjen ha litt fjæring. |

## Hva er ikke testet

Skriptene er testet offline (xacro-generering og -rendering mot `ur_description` for Humble) og planleggingen i mock. Kjøring mot en ekte robot er ikke testet. Står det `not supported by installed macro, ignored: ...` i colcon-loggen, er `ros-humble-ur` gammel, og da bør dere kjøre `sudo apt upgrade`.
