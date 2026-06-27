# CTC Lookup-Table – Konzentrizitäts-Kompensation

> Interne Anleitung zur Konfiguration und zum Tuning des `[ctc]`-Moduls
> (XY-Kompensation des Rundlauffehlers einer synchronisierten Zusatzachse).

---

## 1. Zweck

Das `[ctc]`-Modul korrigiert XY-Bewegungen, um den **Rundlauffehler
(Konzentrizitätsfehler)** eines rotierenden Teils auszugleichen, das an
eine synchronisierte G-Code-Zusatzachse (Standard: `A`) gekoppelt ist.

Das Modul liest laufend die Position dieser Achse und schlägt in einer
**Lookup-Tabelle** nach, wie weit das Teil bei diesem Winkel in X und Y
„auswandert" (`dx`, `dy`). Anschließend verschiebt es die physische
Toolhead-Position um den **Gegenwert**, sodass der Fehler kompensiert
wird:

```
Toolhead-X = logische X − dx(Winkel)
Toolhead-Y = logische Y − dy(Winkel)
```

`dx`/`dy` sind also der **gemessene Fehler**, nicht die Korrektur – die
Korrektur ist das Negative davon und wird vom Modul automatisch
angewendet.

---

## 2. Funktionsprinzip

### 2.1 Stützstellen an echten A-Winkeln (nicht uniform)

Die Tabelle besteht aus **Stützstellen**, die jeweils an einem konkreten
A-Achsen-Winkel sitzen. Die Winkel werden in `lookup_a` angegeben und
**dürfen beliebig (nicht gleichmäßig) verteilt** sein – man referenziert
also direkt die tatsächlich gemessene Achsposition.

| `lookup_a` | `lookup_dx` | `lookup_dy` |
|-----------:|------------:|------------:|
| 0°         | dx₀         | dy₀         |
| 30°        | dx₁         | dy₁         |
| 100°       | dx₂         | dy₂         |
| 250°       | dx₃         | dy₃         |

Die Reihenfolge in der Config ist egal – das Modul sortiert die
Stützstellen intern nach Winkel. Jeder Winkel muss aber **eindeutig**
sein (modulo 360).

### 2.2 Periodizität – unendliche Drehung, eine Umdrehung Tabelle

Die Achse darf sich **beliebig oft** weiterdrehen. Die Auslenkung wird
aber nur für **eine Umdrehung (0–360°)** angegeben, weil sie sich jede
Umdrehung wiederholt. Intern wird jeder Winkel auf `Winkel mod 360`
reduziert:

```
0° = 360° = 720° = 1080° …
90° = 450° = 810° …
305° = 665° = 1025° …
```

Es ist also egal, ob die Achse bei 30° oder bei 3630° steht – es wird
dieselbe Kompensation angewendet. Auch die Interpolation und das Snapping
„wrappen" sauber über die Grenze **360° → 0°** (d. h. zwischen der letzten
und der ersten Stützstelle).

### 2.3 Interpolation oder Snapping

* **`interpolate: True`** (Standard): Zwischen den zwei benachbarten
  Stützstellen wird **linear interpoliert** (inkl. Wrap über 360→0).
* **`interpolate: False`**: Es wird auf die **winkelmäßig nächste**
  Stützstelle gesnappt und exakt deren Wert verwendet (keine Glättung).

---

## 3. Konfiguration

Alle Werte stehen direkt unter der Sektion `[ctc]`. Es gibt keine
Unter-Sektionen.

```ini
[ctc]
axis: A
lookup_a:  0,  30,  100, 250        # Winkel-Stützstellen in Grad (frei verteilbar)
lookup_dx: 0.00, 0.08, -0.05, 0.03  # gemessener Rundlauffehler X je Stützstelle [mm]
lookup_dy: 0.00, 0.02,  0.04, -0.01 # gemessener Rundlauffehler Y je Stützstelle [mm]
interpolate: True
split_delta_xy: 0.01
move_check_distance_axis: 5.0
```

### 3.1 Parameterübersicht

| Option | Pflicht | Default | Bedeutung |
|---|---|---|---|
| `axis` | nein | `A` | G-Code-Buchstabe der getrackten Zusatzachse. Genau **ein** Extra-Achsbuchstabe (nicht X/Y/Z/E/F/N). Muss die Achse sein, die bei `G1`-Moves registriert ist. |
| `lookup_a` | empfohlen | – | Liste der Winkel-Stützstellen in Grad, ein Wert pro Tabelleneintrag. **Nicht-uniform erlaubt**, aber jeder Winkel eindeutig (mod 360). Reihenfolge egal (wird sortiert). Fehlt `lookup_a`, wird ein **gleichmäßiges** 0–360°-Raster angenommen (`i · 360 / Anzahl`). |
| `lookup_dx` | ja* | – | Gemessener Rundlauffehler in **X** [mm], ein Wert pro Stützstelle. Angewandte Korrektur = `−dx`. |
| `lookup_dy` | ja* | – | Gemessener Rundlauffehler in **Y** [mm]. Angewandte Korrektur = `−dy`. Gleiche Anzahl Einträge wie `lookup_a`/`lookup_dx`. |
| `interpolate` | nein | `True` | `True` = linear interpolieren; `False` = auf nächste Stützstelle snappen. |
| `split_delta_xy` | nein | `0.025` mm | Mindeständerung der berechneten XY-Korrektur, ab der ein Move unterteilt wird. Kleiner = feiner/genauer, mehr Moves. Min. `0.01`. |
| `move_check_distance_axis` | nein | kleinster Stützstellen-Abstand | Schrittweite (in Achseinheiten/Grad), in der während einer Achsbewegung auf eine Korrekturänderung geprüft wird. Default = kleinste Lücke zwischen zwei Stützstellen, damit keine übersprungen wird; ohne Tabelle `5.0`. |
| `move_check_distance_a` | nein | – | Legacy-Alias für `move_check_distance_axis` (nicht zusammen mit diesem angeben). |

\* Sind alle `dx`/`dy` = 0 (oder leer), ist die Kompensation **inaktiv**
(reiner Durchlauf).

> **Vorzeichen-Merksatz:** `dx`/`dy` ist der **Fehler**, den man misst.
> Wandert das Teil bei einem Winkel um +0,08 mm in X aus, trägt man
> `0.08` ein – das Modul fährt den Toolhead dann um −0,08 mm, um es
> auszugleichen.

---

## 4. Rechenbeispiel

Tabelle: `lookup_a: 0, 30, 100, 250`, `lookup_dx: 0.0, 1.0, -2.0, 0.5`
(dy = 0), `interpolate: True`.

| A-Position | wirksamer Winkel (mod 360) | interpolierter `dx` | Toolhead-X (bei logisch X=10) |
|---:|---:|---:|---:|
| 30°  | 30°  | 1.00  | 9.00 |
| 65°  | 65°  | −0.50 (Mitte 30↔100) | 10.50 |
| 100° | 100° | −2.00 | 12.00 |
| 305° | 305° | 0.25 (Wrap 250↔360/0) | 9.75 |
| 785° | 65°  | −0.50 | 10.50 |
| 665° | 305° | 0.25  | 9.75 |

Im **Snap-Modus** (`interpolate: False`) würde z. B. `A=40°` auf die
Stützstelle 30° gerundet (`dx = 1.0`), `A=70°` auf 100° (`dx = −2.0`),
`A=340°` auf 0°/360° (`dx = 0.0`).

---

## 5. Werte zur Laufzeit ändern (ohne Neustart)

Zum schnellen Tuning gibt es zwei G-Code-Befehle. Sie ändern die Tabelle
**sofort im RAM** und schreiben **nichts** in die Config-Datei – die
endgültigen Werte trägt man danach manuell unter `[ctc]` ein.

### 5.1 `QUERY_CTC`

Zeigt den aktuellen Zustand an:

```
QUERY_CTC
```

Beispielausgabe:

```
ctc: axis=A active=1 interpolate=True points=4 min_step=30.0000 deg
split_delta_xy=0.0100 mm  move_check_distance_axis=5.0000
lookup_a:  0.0000, 30.0000, 100.0000, 250.0000
lookup_dx: 0.0000, 1.0000, -2.0000, 0.5000
lookup_dy: 0.0000, 0.0000, 0.0000, 0.0000
```

### 5.2 `SET_CTC`

```
SET_CTC [LOOKUP_A=<w,w,...>] [LOOKUP_DX=<v,v,...>] [LOOKUP_DY=<v,v,...>]
        [INTERPOLATE=0|1]
        [INDEX=<i> [A=<wert>] [DX=<wert>] [DY=<wert>]]
        [SPLIT_DELTA_XY=<wert>] [MOVE_CHECK_DISTANCE_AXIS=<wert>]
```

| Parameter | Wirkung |
|---|---|
| `LOOKUP_A`, `LOOKUP_DX`, `LOOKUP_DY` | Ersetzen die jeweilige Spalte komplett. Beim **Ändern der Punktanzahl** muss `LOOKUP_A` mitgegeben werden. |
| `INTERPOLATE=0\|1` | Schaltet zwischen Snapping (`0`) und Interpolation (`1`). |
| `INDEX=<i>` + `A`/`DX`/`DY` | Ändert **eine einzelne** Stützstelle (Index in der sortierten Tabelle, beginnend bei 0). |
| `SPLIT_DELTA_XY` | Setzt die Unterteilungs-Schwelle (≥ 0.01). |
| `MOVE_CHECK_DISTANCE_AXIS` | Setzt die Prüf-Schrittweite (≥ 0.01). |
| *(ohne Parameter)* | Gibt nur den aktuellen Zustand aus (wie `QUERY_CTC`). |

### 5.3 Beispiele

```ini
# Komplette Tabelle setzen (alle drei Spalten zusammen):
SET_CTC LOOKUP_A=0,30,100,250 LOOKUP_DX=0,1,-2,0.5 LOOKUP_DY=0,0,0,0

# Nur dx/dy bei gleicher Punktanzahl ersetzen (Winkel bleiben):
SET_CTC LOOKUP_DX=0,0.9,-1.8,0.4 LOOKUP_DY=0,0,0,0

# Einzelne Stützstelle anpassen (Index 2 = 100°):
SET_CTC INDEX=2 DX=-1.7
SET_CTC INDEX=2 A=110 DX=-1.7 DY=0.02   # auch den Winkel verschieben

# Interpolation umschalten:
SET_CTC INTERPOLATE=0      # nearest / snap
SET_CTC INTERPOLATE=1      # linear

# Sampling/Unterteilung feinjustieren:
SET_CTC SPLIT_DELTA_XY=0.02 MOVE_CHECK_DISTANCE_AXIS=10

# Kompensation deaktivieren (Tabelle leeren):
SET_CTC LOOKUP_DX="" LOOKUP_DY=""

# Aktuellen Zustand anzeigen:
SET_CTC
```

> ⚠️ **Wichtig:** G-Code trennt Argumente an Leerzeichen. Listen daher als
> **ein Token ohne Leerzeichen** übergeben: `LOOKUP_DX=0,1,-2,0.5`
> (richtig) – **nicht** `LOOKUP_DX=0, 1, -2, 0.5` (falsch).
> In der **Config-Datei** sind Leerzeichen dagegen erlaubt.

---

## 6. Werte im Status auslesen

Die aktuellen Werte stehen auch im Status-Objekt zur Verfügung (z. B. in
Makros oder der Mainsail/Fluidd-Konsole):

| Feld | Inhalt |
|---|---|
| `printer.ctc.axis` | getrackte Achse |
| `printer.ctc.active` | Kompensation aktiv (True/False) |
| `printer.ctc.interpolate` | Interpolationsmodus |
| `printer.ctc.points` | Anzahl Stützstellen |
| `printer.ctc.min_step` | kleinster Stützstellen-Abstand [°] |
| `printer.ctc.split_delta_xy` | Unterteilungs-Schwelle [mm] |
| `printer.ctc.move_check_distance_axis` | Prüf-Schrittweite |
| `printer.ctc.lookup_a` | Liste der Winkel |
| `printer.ctc.lookup_dx` / `lookup_dy` | Fehler-Listen |

---

## 7. Tuning-Workflow (Empfehlung)

1. Achse als G-Code-Achse registrieren (z. B. per `MANUAL_STEPPER ...
   GCODE_AXIS=A`).
2. Pro relevantem Winkel den Rundlauffehler messen und als `dx`/`dy`
   notieren (Vorzeichen = Richtung der Auswanderung).
3. Werte per `SET_CTC` einspielen und direkt testen – ohne Neustart.
4. Einzelne Stützstellen per `SET_CTC INDEX=… DX=… DY=…` nachziehen.
5. Wenn das Ergebnis passt: `QUERY_CTC` ausführen und die angezeigten
   `lookup_a`/`lookup_dx`/`lookup_dy` dauerhaft in die `[ctc]`-Sektion der
   `printer.cfg` übertragen.

---

## 8. Stolperfallen

* **Leerzeichen in Listen** beim G-Code-Befehl vermeiden (siehe 5.3).
* **Punktanzahl geändert** → beim `SET_CTC` immer `LOOKUP_A` mitgeben.
* **Doppelte Winkel** (auch `0` und `360`, da mod 360 identisch) werden
  abgelehnt – jede Stützstelle braucht einen eindeutigen Winkel.
* **`dx`/`dy` ist der Fehler, nicht die Korrektur** – das Modul invertiert
  selbst.
* `SET_CTC`-Änderungen sind **flüchtig** und gehen beim Neustart verloren,
  bis sie in der Config stehen.
