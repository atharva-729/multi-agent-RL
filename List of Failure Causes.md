### Failure Causes (C)
| Code   | Name                         | Description                                                                                                               | Detectability (D) | Ranking       | Reasoning                                                                                                                                       |
| ------ | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------- | ----------------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **C1** | Uneven or Obstructed Terrain | Natural bumps, pits, tall grass, or surface debris like twigs that disrupt smooth wheel movement or cause heading change. | **4**             | **4**         | RPM drop and `gyro.x/y` tilt change can help detect terrain irregularities, but effectiveness depends on motion dynamics and sensor resolution. |
| **C2** | Slipping Hazards             | Wet or low-friction surfaces like dew, mud, or wet foliage cause wheel slip — wheels spin but robot doesn’t move.         | **3**             | **2**         | Slipping is well-detectable using encoder–accelerometer mismatch logic. High RPM without actual motion is a strong slip indicator.              |
| **C3** | Communication Loss           | Robot loses Bluetooth contact with controller (e.g., when beyond range or due to interference).                           | **4**             | **3**         | HC-05 doesn’t indicate disconnect, but a software-side timeout on message validity can reliably signal comms loss within seconds.               |
| **C4** | Dynamic Obstacles            | Humans, animals, or other moving robots unpredictably obstruct the robot’s path.                                          | **8**             | **7**         | Detection depends on the dynamic obstacle being in the ultrasonic sensor’s range and direction. Blind spots reduce reliability.                 |
| **C5** | Static Obstacles             | Fixed, unmoving barriers such as walls, fences, benches, or poles.                                                        | **2**             | **1**         | Static obstacles are reliably detected with front-facing ultrasonic sensors and mapping. False negatives are rare.                              |
| **C6** | Amplified Hazard Zones       | Terrain regions where the robot can enter but becomes trapped due to geometry or incline.                                 | **9**             | **8**         | No sensor can detect “trapped geometry” before entry. It’s only known after the robot repeatedly fails to exit.                                 |
| **C7** | Water Damage Zones           | Surface water threatens to leak into electronics, causing irreversible component failure.                                 | **10**            | **9**         | No sensors currently detect surface water presence. Damage is only known after it occurs. Completely undetectable pre-failure.                  |
| **C8** | Intra-agent Interference     | Conflicting interpretations or decisions between agents due to sensor inconsistency or task assignment logic.             | **7**             | **6**         | No centralized fusion or conflict resolution logic — cause only seen after failure manifests.                                                   |
| **C9** | Coordination Breakdown       | One robot fails and others do not redistribute its remaining tasks, leading to coverage failure.                          | **6**             | **5**         | Detection is possible by tracking robot states and active task queues — but only if monitoring systems exist.                                   |

-----

### Failure Modes (F)


| Code   | Mode              | Description                                                                                                                                                        |
| ------ | ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **F1** | Veers Off         | The robot deviates from the intended path or heading due to slipping, terrain, or incorrect sensor feedback. It may still move but ends up in the wrong direction. |
| **F2** | Gets Stuck        | The robot is unable to proceed physically due to an obstruction, poor terrain, or loss of traction. It may remain powered but make no forward progress.            |
| **F3** | Rollover          | The robot tips over and cannot recover its upright orientation, caused by unstable terrain or abrupt heading changes. This may stop all further movement.          |
| **F4** | Stops Functioning | The robot shuts down or becomes completely unresponsive due to motor failure, electronics damage, or system-level errors. It halts all operations.                 |
| **F5** | Deadlock          | The robot’s control logic enters a loop or conflict state that prevents action. Unlike F4, the robot may still respond but does nothing meaningful.                |
| **F6** | Path Conflict     | Two robots attempt to occupy the same space or path segment due to poor coordination. Results in blocking, waiting loops, or mutual obstruction.                   |



---

### Failure Effects (E)

| Code    | Effect Name                                           | Description                                                                           | Severity (S) | Ranking |
| ------- | ----------------------------------------------------- | ------------------------------------------------------------------------------------- | ------------ | ------- |
| **E1**  | Robot completes task but with delay                   | Task completed successfully but with minor delay; no hardware damage                  | **2**            | **1**  |
| **E2**  | Robot completes task with recoverable damage          | Task completed, but robot suffers minor damage (e.g., wheel shaft stress)             | **4**            | **2**  | 
| **E3**  | Robot completes task with irrepairable damage               | Robot performs task but suffers irrecoverable damage at the end (e.g., short circuit) | **9**            | **6**  |
| **E4**  | Robot has partial coverage, no damage                  | Partial task coverage; robot continues functioning; no repair needed                  | **5**            | **3**  |
| **E5**  | Robot has partial coverage, has repairable damage               | Partial coverage + robot suffers recoverable hardware damage (e.g., jam, overheating) | **6**            | **4**  |
| **E6**  | Robot has permanent damage and mission failure          | Task aborted; robot is physically bricked and unrecoverable                           | **10**           | **7**  |
| **E7**  | Robot has no damage and mission failure                  | Logic failure, deadlock, or control bug causes mission abort; hardware still intact   | **7**            | **5**  |
| **E8**  | Robot requires human help to finish                   | Task was not completed autonomously; human had to intervene or reposition robot       | **8**            | **6**  |



### P(Cause → Failure Mode)

| Cause → Mode              | F1 Veers Off | F2 Gets Stuck | F3 Rollover | F4 Stops Functioning | F5 Deadlock | F6 Path Conflict |
|--------------------------|--------------|----------------|--------------|-----------|--------------|-------------------|
| **C1** Uneven Terrain     | 0.40         | 0.45           | 0.05         | 0.00      | 0.00         | 0.10              |
| **C2** Slipping Hazard    | 0.35         | 0.50           | 0.00         | 0.00      | 0.10         | 0.05              |
| **C3** Communication Loss | 0.00         | 0.00           | 0.00         | 0.70      | 0.20         | 0.10              |
| **C4** Dynamic Obstacles  | 0.15         | 0.50           | 0.00         | 0.00      | 0.00         | 0.35              |
| **C5** Static Obstacles   | 0.10         | 0.60           | 0.00         | 0.00      | 0.10         | 0.20              |
| **C6** Hazard Zones       | 0.00         | 0.45           | 0.10         | 0.00      | 0.45         | 0.00              |
| **C7** Water Damage Zones | 0.00         | 0.00           | 0.00         | 0.90      | 0.00         | 0.10              |
| **C8** Intra-agent Interference| 0.25         | 0.00           | 0.00         | 0.00      | 0.30         | 0.45              |
| **C9** Coordination Breakdown  | 0.00         | 0.00           | 0.00         | 0.00      | 0.90         | 0.10              |

### **Notes:**

#### **C1: Uneven/Obstructed Terrain**

* `F1: 0.40` – Heading shifts due to side-stuck wheels on stones, pits
* `F2: 0.45` – Grass or twigs entangle shaft → real observed stalls
* `F3: 0.05` – Rollover *possible* on extreme terrain (not in lawn)
* `F4: 0.00` – Robot didn’t freeze completely
* `F5: 0.00` – No system-wide lock
* `F6: 0.10` – Veering may land robot into teammate’s zone

#### **C2: Slipping Hazard**

* `F1: 0.40` – Uneven slippage (one wheel) leads to veer
* `F2: 0.50` – Most common result → robot spins but doesn’t move
* `F3: 0.00` – Slipping ≠ tipping
* `F4: 0.00` – Doesn’t freeze
* `F5: 0.10` – Retry loop resembles soft deadlock
* `F6: 0.00` – No path conflict observed

#### **C3: Communication Loss**

* `F4: 0.70` – Full stop on BT loss, no fallback logic yet
* `F5: 0.20` – If multiple robots freeze, coordination can hang
* `F6: 0.10` – Dead robot in grid may block others
* All others: `0.00` – No motion, no heading, no terrain involvement

#### **C4: Dynamic Obstacles**

* `F1: 0.15` – Unintended heading shift possible when reacting (no logic)
* `F2: 0.50` – Gets stuck waiting for obstacle to clear
* `F6: 0.35` – High chance of robot-robot interaction blocking path
* All others: `0.00` – No tipping, freezing, or global deadlock observed

#### **C5: Static Obstacles**

* `F1: 0.10` – Wheel hits side = slight veer
* `F2: 0.60` – Most frequent: robot keeps trying but fails to move
* `F5: 0.10` – Future risk if you add retry logic without timeout
* `F6: 0.20` – Teammates may approach the blocked path
* All others: `0.00`

#### **C6: Amplified Hazard Zones**

* `F2: 0.45` – Robot can’t escape → main issue
* `F3: 0.20` – Risk of rollover due to terrain shape
* `F5: 0.30` – Logic timeout (you plan to add this) → deadlock state
* `F1: 0.05` – Slight directional shift during escape attempts
* All others: `0.00`

#### **C7: Water Damage Zones**

* `F4: 0.90` – Electronics dead. Immediate.
* `F6: 0.10` – Dead body may block path
* All others: `0.00` – No motion, no recovery, no logic after entry

---

### P(Failure Mode → Failure Effect)

| Failure Mode ↓ / Effect →             | E1 complete + Delay + no damage | E2 complete + Delay + repairable | E3 complete + Delay + irrepairable | E4 Partial coverage + no damage | E5 Partial coverage + Repairable | E6 mission faliure + irrepairable | E7 mission failure + no damage | E8 Human Help |
|---------------------------------------|------------------------|-----------------|--------------|------------|-------------------|------------------------|---------------|----------------|
| **F1** Veers Off                      | 0.50     | 0.00            | 0.00         | 0.30       | 0.00              | 0.00                   | 0.00          | 0.20           |
| **F2** Gets Stuck                     | 0.20     | 0.25            | 0.00         | 0.05       | 0.15              | 0.05                   | 0.00          | 0.30           |
| **F3** Rollover                       | 0.00     | 0.00            | 0.00         | 0.00       | 0.00              | 0.70                   | 0.00          | 0.30           |
| **F4** Stops Functioning              | 0.00     | 0.00            | 0.05         | 0.00       | 0.00              | 0.00                   | 0.75          | 0.20           |
| **F5** Deadlock                       | 0.00     | 0.00            | 0.00         | 0.00       | 0.05              | 0.20                   | 0.70          | 0.05           |
| **F6** Path Conflict                  | 0.10     | 0.15            | 0.05         | 0.00       | 0.25              | 0.25                   | 0.10          | 0.10           |
---

### **Notes**


#### ✅ **F1 – Veers Off**

* **E1**: Task completes with delay after model corrects heading
* **E2**: No mechanical stress → no recoverable damage
* **E3**: Never reaches task completion *and* dies from veering
* **E4**: Veer may skip cells unintentionally → partial coverage
* **E5**: No damage involved
* **E6**: Doesn’t escalate to total mission loss
* **E7**: Logic stays fine; just heading error
* **E8**: May enter repetitive veer loop → requires manual fix

---

#### ✅ **F2 – Gets Stuck**

* **E1**: Task gets reassigned after stuck detection → just delay
* **E2**: Retry motion strains motors → recoverable damage
* **E3**: Doesn’t complete task before dying
* **E4**: Rare chance task coverage is ambiguous or missed
* **E5**: Damage + partial task due to failed retry → both present
* **E6**: Robot might burn out if stuck too long
* **E7**: Model handles logic fine → no logical failure
* **E8**: Still common to nudge robot or restart manually

---

#### ✅ **F3 – Rollover**

* **E1**: Robot dies instantly — no delays possible
* **E2**: Rare case of light tipping with no permanent damage
* **E3**: If robot flips after completing task → post-task death
* **E4**: Doesn’t miss task quietly — usually a full stop
* **E5**: No partial + repair case fits rollover failure
* **E6**: Primary outcome — bricked bot and mission failure
* **E7**: Hardware fault, not logic → no deadlock
* **E8**: If tipped early, human might flip it back

---

#### ✅ **F4 – Stops Functioning**

* **E1**: Robot freezes — no continuation or delay
* **E2**: Typically no damage involved in stops
* **E3**: May finish task before comms cut out
* **E4**: Doesn’t cause partial coverage directly
* **E5**: No damage involved
* **E6**: Stops aren’t physical failures → no permanent damage
* **E7**: Main cause: logic/comms failure
* **E8**: Bringing it into range often revives it

---

#### ✅ **F5 – Deadlock**

* **E1**: System never recovers — not just delayed
* **E2**: No retries or movement → no stress → no repair case
* **E3**: Deadlock happens mid-task → never completes
* **E4**: Coverage is not partial — whole task stalls
* **E5**: If pit/retry causes minor damage before freezing
* **E6**: Some deadlocks arise from retrying until breakdown
* **E7**: Core outcome — logic fails, no damage
* **E8**: Rare chance a human could reset it

---

#### ✅ **F6 – Path Conflict**

* **E1**: Two robots get in each other’s way → slight delay
* **E2**: Conflict may lead to motor/bump damage
* **E3**: One finishes task but dies after collision
* **E4**: Not a partial coverage scenario
* **E5**: Blocking + damage + uncompleted task → all possible
* **E6**: Conflict escalates to crash → mission and bot both lost
* **E7**: If neither robot exits path → logic deadlock
* **E8**: Manual reset may be needed if planner can’t resolve

---


### to calculate O for RPN calculation

* a cell is given assigned a cause of failure
* this cause can lead to many failure modes, and these modes can lead to many failure effects
* for RPN, we need O (probability of occurence of that cause) 
* better to calculate O using Bayes' theorem, where you consider the contributions of every single mode and effect
* rather than getting the occurence of the most likely mode (or effect (?)), this also reduces bias
* the probabilities C to F and F to E are used to calculate O
* this way, every cell having the same cause will have the same O