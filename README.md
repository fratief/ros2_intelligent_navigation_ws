# 🤖 ROS2 Intelligent Navigation Workspace

Workspace ROS2 dedicato allo sviluppo di un robot autonomo in grado di:

- percepire l’ambiente tramite LaserScan
- riconoscere ed evitare ostacoli
- navigare in modo reattivo e intelligente
- integrare in futuro moduli di AI e machine learning

Il progetto nasce come controller reattivo per TurtleBot3, ma è pensato per essere esteso a robot più complessi.

---

##  Funzionalità principali

###  RiskModel (Percezione)

Modulo dedicato all’elaborazione del LaserScan:

- suddivisione in settori (front, left, right, back)
- calcolo del rischio locale e globale
- smoothing e filtraggio
- output pulito per FSM e controller

### FSM (Finite State Machine)

Gestisce i comportamenti principali:

- **EXPLORE** → esplorazione casuale controllata
- **TRANSITION** → movimento di transizione dopo un evitamento
- **ESCAPE** → evitamento ostacoli reattivo

###  Behaviors (Comportamenti)

Attualmente i comportamenti sono implementati **all’interno di `turtle_controller.cpp`**:

- `handle_escape()`
- `handle_transition()`
- `handle_explore()`

In futuro potranno essere estratti in un modulo dedicato (`behaviors.cpp` / `behaviors.hpp`).

###  Controller

Genera i comandi di movimento (Twist) combinando:

- rischi dal RiskModel
- stato della FSM
- comportamenti attivi
- logiche di movimento reattivo

---

##  Struttura del progetto

<pre>
ros2_intelligent_navigation_ws/
│
├── src/
│   └── turtle_controller/
│       ├── include/turtle_controller/
│       │   ├── risk_model.hpp
│       │   ├── fsm.hpp
│       │  
│       ├── src/
│       │   ├── risk_model.cpp
│       │   ├── fsm.cpp
│       │   └── turtle_controller.cpp   ← contiene anche i behaviors
│       ├── CMakeLists.txt
│       └── package.xml
│
├── .gitignore
└── README.md
</pre>
---

## Installazione

Assicurati di avere ROS2 (Humble o successivo) installato.

```bash
cd ros2_intelligent_navigation_ws
colcon build
source install/setup.bash
```

---

## Esecuzione

### Avvio del controller

```bash
ros2 run turtle_controller turtle_controller_node
```

Requisiti per il funzionamento:

  - un nodo LaserScan attivo (TurtleBot3 reale o simulazione)

  - un topic /scan pubblicato regolarmente

  - parametri di velocità configurati nel controller

  - ROS2 Humble (o successivo) correttamente installato

## Comportamento atteso

- il robot esplora l’ambiente in modo autonomo

- evita ostacoli in tempo reale

- modifica automaticamente la velocità in base al rischio:

    - alta velocità in zone sicure

    - velocità ridotta in presenza di rischio moderato

    - stop + manovra di fuga in caso di rischio elevato

- transizioni fluide tra stati (EXPLORE → ESCAPE → TRANSITION)


## 🧭 Obiettivi futuri

### 🔹 1. Miglioramenti al controllo
- controllo automatico della velocità basato su:
  - distanza dagli ostacoli
  - rischio globale
  - direzione di movimento
- tuning dinamico dei parametri:
  - PID per velocità lineare/angolare
  - smoothing dei comandi
  - soglie di rischio adattive
- gestione più fluida delle transizioni tra stati
- riduzione delle oscillazioni e dei movimenti inutili

### 🔹 2. Navigazione intelligente
- aggiunta di un **Goal Manager** per raggiungere posizioni target
- integrazione di un **planner locale**, come:
  - DWA (Dynamic Window Approach)
  - VFH (Vector Field Histogram)
  - RRT / RRT*
- generazione di traiettorie più morbide e prevedibili
- creazione di una **mappa del rischio dinamica**
- comportamento ibrido:
  - reattivo (RiskModel)
  - deliberativo (planner)

### 🔹 3. Intelligenza Artificiale
- modelli di **Reinforcement Learning** per:
  - evitare ostacoli in modo più efficiente
  - imparare strategie di navigazione
  - ottimizzare velocità e traiettorie
- modelli di **Imitation Learning** basati su dimostrazioni umane
- classificazione degli ostacoli tramite AI (in futuro con camera RGB o depth)
- integrazione di reti neurali leggere (TensorRT, ONNX Runtime)

### 🔹 4. Estensione a robot reali
- supporto a robot diversi da TurtleBot3
- calibrazione automatica dei sensori
- moduli di sicurezza avanzati:
  - emergency stop
  - riduzione velocità in aree critiche
  - monitoraggio continuo del rischio

