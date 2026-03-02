Read [toy\_version\_2.txt](vscode-file://vscode-app/snap/code/226/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)

Here's a full breakdown:

### Header: `6 10 300 3 0`


| Value  | Meaning                                   |
| ------ | ----------------------------------------- |
| N=6    | 6 junctions (0–5)                        |
| M=10   | 10 streets (0–9)                         |
| T=300  | Vehicles have 300 seconds max route time  |
| C=3    | 3 vehicles                                |
| S=0    | Depot is junction 0                       |
| (no W) | W defaults to 0 — no water waste penalty |

### Streets (10 lines, indices 0–9):


| Street | A→B     | Dir         | Time(s) | Length(m) | Cat           | Req(L/km)      | Notes                      |
| ------ | -------- | ----------- | ------- | --------- | ------------- | -------------- | -------------------------- |
| 0      | 1→5     | one-way     | 5       | 81        | Optional      | 30 (Heavy)     | Only L can clean           |
| 1      | 0→1     | one-way     | 24      | 0         | Connector     | 0              | Travel only                |
| 2      | 1↔4     | two-way     | 114     | 0         | Connector     | 0              | Travel only                |
| 3      | 0→2     | one-way     | 170     | 266       | Optional      | 10 (Light)     | Any vehicle can clean      |
| 4      | 3↔4     | two-way     | 160     | 0         | Connector     | 0              | Travel only                |
| 5      | **0→4** | **one-way** | **179** | **76**    | **Mandatory** | **30 (Heavy)** | **Must clean, only L can** |
| 6      | 2→5     | one-way     | 130     | 467       | Optional      | 10 (Light)     | Any vehicle can clean      |
| 7      | **0↔5** | **two-way** | **156** | **264**   | **Mandatory** | **30 (Heavy)** | **Must clean, only L can** |
| 8      | 1↔3     | two-way     | 123     | 361       | Optional      | 20 (Medium)    | M or L can clean           |
| 9      | 1→2     | one-way     | 165     | 421       | Optional      | 10 (Light)     | Any vehicle can clean      |

### Fleet: [S L M](vscode-file://vscode-app/snap/code/226/usr/share/code/resources/app/out/vs/code/electron-browser/workbench/workbench.html)


| Vehicle | Type       | Capacity | Can clean            |
| ------- | ---------- | -------- | -------------------- |
| 0       | Small (S)  | 10 L/km  | Light only           |
| 1       | Large (L)  | 30 L/km  | Light, Medium, Heavy |
| 2       | Medium (M) | 20 L/km  | Light, Medium        |

### Why it's infeasible

Both mandatory streets (5 and 7) require **30 L/km** — only the **Large** vehicle can clean them.

**Mandatory street 5** (0→4, one-way, 179s): The Large vehicle goes depot(0)→4 in 179s. Now it's at junction 4. To return to depot(0), the only paths from 4 are:

* 4↔1 (114s) → 1 has no arc to 0 (street 1 is one-way 0→1)
* 4↔3 (160s) → 3↔1 (123s) → still stuck at 1

There is **no path from junction 4 back to depot 0** using one-way directions. So street 5 alone is impossible.

**Mandatory street 7** (0↔5, 156s): Going 0→5 takes 156s, then returning 5→0 takes another 156s = 312s > 300s. Going via other junctions is even longer. So cleaning street 7 **round-trip exceeds the 300s budget**.

Since one vehicle must clean both mandatory streets and neither can be served individually, the instance is **mathematically infeasible**.
